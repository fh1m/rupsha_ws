"""RupshaMission -- the mission language.

A mission is a Python file with `def run(rupsha, log)`. Every verb in
manifest/rupsha.yaml is a method here with the manifest's fields as keyword
arguments and its defaults filled in:

    rupsha.arm()
    rupsha.goto_gnss(lat=..., lon=..., radius=2)
    rupsha.turn_to(heading=90)
    rupsha.move_forward(duration=3, speed=0.5)
    rupsha.search_spiral(tag_id=4, radius=15)
    rupsha.vision_approach(tag_id=4, standoff=1.5)
    rupsha.add_marker_leg(lat=..., lon=..., tag_id=4); rupsha.start_autonomy()
    rupsha.stop(); rupsha.disarm()

Verbs return the result (`.success`, `.outcome`, `.message`) and NEVER raise on
an operator-causable failure; the mission decides. Queries between verbs:

    rupsha.detected(tag_id=4)            # bool, current frame
    rupsha.wait_for(tag_id=4, timeout=8) # bool, blocks until seen or timeout
    rupsha.where(tag_id=4)               # 'left' | 'center' | 'right' | 'unknown'
    rupsha.here()                        # (lat, lon), NaN when no fix
    rupsha.head()                        # heading deg, NaN when absent

Capability, budget and plan (mongla's, same words):

    if rupsha.can('vision_approach'): ...
    rupsha.require('goto_gnss', 'search_spiral')     # refuse before moving
    rupsha.use_budget(1800, reserve_s=120)
    rupsha.run_plan([rupsha.step('leg1', points=..., worst_case_s=..., run=fn), ...])
"""

from __future__ import annotations

import math

from rupsha_control.manifest import VERBS

CENTER_DEG = 8.0


class MissionRefused(RuntimeError):
    """A `require()` failed. Raised before the vehicle moves, never during."""


class TaskAbandoned(RuntimeError):
    """A step overran its deadline; run_plan catches it and moves on."""


def _make_verb(verb: str):
    spec = VERBS[verb]
    fields, defaults = spec['fields'], spec['defaults']

    def method(self, *args, **kwargs):
        if len(args) > len(fields):
            raise TypeError(f'{verb}() takes {len(fields)} positional fields: {fields}')
        kw = dict(defaults)
        kw.update(zip(fields, args, strict=False))
        unknown = set(kwargs) - set(fields)
        if unknown:
            raise TypeError(f'{verb}() has no field(s) {sorted(unknown)}; fields are {fields}')
        kw.update(kwargs)
        return self._do(verb, kw)
    method.__name__ = verb
    method.__doc__ = f'{spec["help"]}\n\nfields: {", ".join(fields) or "none"}'
    return method


class RupshaMission:

    def __init__(self, port, log=None, supported=None):
        self.port = port
        self.log = log
        self._supported = supported            # None = every manifest verb
        self.results = []
        self._deadline = math.inf
        self._budget_end = math.inf
        self._reserve = 0.0
        self.scorecard = []

    # ── dispatch ─────────────────────────────────────────────────────────────
    def _say(self, text):
        if self.log:
            self.log.info(text)

    def _do(self, verb, kw):
        if not self.can(verb):
            raise MissionRefused(f'{verb} is not available on this port')
        if self.port.now() > self._deadline:
            raise TaskAbandoned(f'step deadline passed before {verb}')
        res = self.port.run(verb, **kw)
        self.results.append((verb, kw, res))
        self._say(f'[{verb}] {res.outcome.name}: {res.message}')
        return res

    # ── capability ───────────────────────────────────────────────────────────
    def can(self, verb: str) -> bool:
        if verb not in VERBS:
            return False
        return self._supported is None or verb in self._supported

    def require(self, *verbs: str) -> None:
        missing = [v for v in verbs if not self.can(v)]
        if missing:
            raise MissionRefused(f'mission needs {missing}, which this port cannot run')

    # ── queries ──────────────────────────────────────────────────────────────
    def _hits(self, target_name='', tag_id=-1, fresh_s=0.5):
        now = self.port.now()
        out = []
        for d in self.port.detections():
            if now - d.stamp > fresh_s:
                continue
            if tag_id >= 0 and d.tag_id != tag_id:
                continue
            if tag_id < 0 and target_name and d.label != target_name:
                continue
            if tag_id < 0 and not target_name and d.label != 'tag':
                continue
            out.append(d)
        return out

    def detected(self, target_name='', tag_id=-1) -> bool:
        return bool(self._hits(target_name, tag_id))

    def wait_for(self, target_name='', tag_id=-1, timeout=5.0) -> bool:
        end = self.port.now() + timeout
        while self.port.now() < end:
            if self.detected(target_name, tag_id):
                return True
            self.port.sleep(0.1)
        return False

    def where(self, target_name='', tag_id=-1) -> str:
        hits = self._hits(target_name, tag_id)
        if not hits:
            return 'unknown'
        b = max(hits, key=lambda d: d.confidence).bearing_deg
        return 'center' if abs(b) <= CENTER_DEG else ('right' if b > 0 else 'left')

    def here(self):
        t = self.port.state()
        return t.lat, t.lon

    def head(self) -> float:
        return self.port.state().yaw_deg

    def sleep(self, seconds: float) -> None:
        self.port.sleep(seconds)

    # ── budget and plan ──────────────────────────────────────────────────────
    def use_budget(self, total_s: float, reserve_s: float = 0.0) -> None:
        self._budget_end = self.port.now() + total_s
        self._reserve = reserve_s

    def remaining(self) -> float:
        return self._budget_end - self._reserve - self.port.now()

    @staticmethod
    def step(name, *, points, worst_case_s, run, fallback=None, fallback_s=0.0,
             fallback_points=0, needs=''):
        return {'name': name, 'points': int(points), 'worst_case_s': float(worst_case_s),
                'run': run, 'fallback': fallback, 'fallback_s': float(fallback_s),
                'fallback_points': int(fallback_points), 'needs': needs}

    def run_plan(self, steps, order='as_written'):
        """Run declared steps, deciding full / fallback / skip from the live clock each time."""
        todo = list(steps)
        if order == 'by_value':
            todo.sort(key=lambda s: -s['points'] / max(1.0, s['worst_case_s']))
        for s in todo:
            left = self.remaining()
            if s['needs'] and not self.can(s['needs']):
                self.scorecard.append((s['name'], 'skipped', f'{s["needs"]} unavailable'))
                continue
            if s['worst_case_s'] <= left:
                fn, budget, how = s['run'], s['worst_case_s'], 'full'
            elif s['fallback'] and s['fallback_s'] <= left:
                fn, budget, how = s['fallback'], s['fallback_s'], 'fallback'
            else:
                self.scorecard.append((s['name'], 'skipped', f'{left:.0f} s left'))
                continue
            self._deadline = self.port.now() + budget
            t0 = self.port.now()
            try:
                fn(self)
                verdict = how
            except TaskAbandoned as exc:
                verdict = f'abandoned ({exc})'
                self.port.run('stop')
            finally:
                self._deadline = math.inf
            self.scorecard.append((s['name'], verdict, f'{self.port.now() - t0:.0f} s'))
        return self.scorecard


for _verb in VERBS:
    setattr(RupshaMission, _verb, _make_verb(_verb))
