"""Which detectors run, chosen by what the executive is doing.

The Pi 4 cannot run ArUco and YOLO on every frame and still steer from them.
It does not need to: driving a GNSS leg only needs to notice a post going by,
approaching a marker never needs YOLO, and reversing off an obstacle needs
nothing. Two tables, both pure data:

    AutonomyState NAME --STATE_PHASE--> phase --PHASE_DETECTORS--> {detectors}

The executive's state arrives as the NAME of `rupsha_control.manifest.
AutonomyState` (the node converts the wire ordinal), so this module needs
neither ROS nor the manifest. An unknown name raises: a state added to the
manifest and not here must be decided by a person, not defaulted to "run
everything" or "run nothing".
"""
from __future__ import annotations

from collections.abc import Iterable, Mapping
from collections.abc import Set as AbstractSet

ARUCO = 'aruco'
OBJECTS = 'objects'
DETECTORS: frozenset[str] = frozenset({ARUCO, OBJECTS})

PHASE_DETECTORS: Mapping[str, frozenset[str]] = {
    'idle': frozenset(),
    # A post may come into view before the leg's GNSS goal; ArUco is cheap.
    'navigating': frozenset({ARUCO}),
    'search': frozenset({ARUCO, OBJECTS}),
    'approach_marker': frozenset({ARUCO}),
    'approach_object': frozenset({OBJECTS}),
    'verify_position': frozenset(),
    'verify_marker': frozenset({ARUCO}),
    'verify_object': frozenset({OBJECTS}),
    # Backing off or stuck: nothing vision says changes what happens next.
    'reversing': frozenset(),
    'stuck': frozenset(),
}

STATE_PHASE: Mapping[str, str] = {
    'IDLE': 'idle',
    'NAVIGATING': 'navigating',
    'SEARCH_PATTERN': 'search',
    'APPROACHING_MARKER': 'approach_marker',
    'APPROACHING_OBJECT': 'approach_object',
    'VERIFYING_POSITION': 'verify_position',
    'VERIFYING_MARKER': 'verify_marker',
    'VERIFYING_OBJECT': 'verify_object',
    'REVERSING': 'reversing',
    'STUCK': 'stuck',
}


def phase_for_state(state_name: str,
                    table: Mapping[str, str] = STATE_PHASE) -> str:
    try:
        return table[state_name]
    except KeyError:
        raise KeyError(f'AutonomyState {state_name!r} has no vision phase; add it '
                       f'to rupsha_vision.scheduler.STATE_PHASE') from None


def detectors_for_phase(phase: str,
                        table: Mapping[str, AbstractSet[str]] = PHASE_DETECTORS
                        ) -> frozenset[str]:
    try:
        return frozenset(table[phase])
    except KeyError:
        raise KeyError(f'unknown vision phase {phase!r}; one of '
                       f'{sorted(table)}') from None


def schedule(state_name: str,
             available: Iterable[str] | None = None
             ) -> tuple[frozenset[str], frozenset[str]]:
    """`(run, wanted_but_unavailable)` for an executive state.

    `available` is the set of detectors this node actually has (None = all).
    The second set is returned rather than silently dropped so the caller can
    say, loudly, that the phase asked for something it cannot do.
    """
    wanted = detectors_for_phase(phase_for_state(state_name))
    have = DETECTORS if available is None else frozenset(available)
    return wanted & have, wanted - have


def check_tables(state_names: Iterable[str]) -> None:
    """Raise unless every given state maps to a phase and every phase only names
    known detectors. Called with the manifest's AutonomyState at node start."""
    missing = sorted(set(state_names) - set(STATE_PHASE))
    if missing:
        raise KeyError(f'AutonomyState values with no vision phase: {missing}')
    for phase, dets in PHASE_DETECTORS.items():
        bad = set(dets) - DETECTORS
        if bad:
            raise ValueError(f'phase {phase!r} names unknown detectors {sorted(bad)}')
    for state, phase in STATE_PHASE.items():
        if phase not in PHASE_DETECTORS:
            raise KeyError(f'state {state!r} maps to undefined phase {phase!r}')
