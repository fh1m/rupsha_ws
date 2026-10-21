"""Make `import rupsha_*` resolve to THIS checkout, not a sourced install tree.

Sourcing a colcon workspace puts `install/<pkg>/lib/python3.12/site-packages` on
PYTHONPATH, and `import rupsha_control` then resolves there. In a worktree or on
the Pi after an edit, the suite goes green about a file nobody is running. This
runs before any test module imports, puts every `src/<pkg>/` first and drops the
install/build entries that would shadow it (mongla's conftest, same reasoning).

`rupsha_interfaces` and `rupsha_sim` are exempt by construction: they have no
`src/<name>/<name>/` Python source, so their install paths are never evicted.
"""
import pathlib
import sys

_ROOT = pathlib.Path(__file__).resolve().parent
_SRC = _ROOT / 'src'


def _source_packages():
    if not _SRC.is_dir():
        return {}
    return {d.name: d for d in sorted(_SRC.iterdir())
            if d.is_dir() and (d / d.name).is_dir()}


def _install_shadow(entry: str, names) -> bool:
    if '/install/' not in entry and '/build/' not in entry:
        return False
    return any(f'/{n}/' in entry for n in names)


def _pin_to_source():
    pkgs = _source_packages()
    if not pkgs:
        return []
    names = set(pkgs)
    sys.path[:] = [p for p in sys.path if not _install_shadow(str(p), names)]
    for path in pkgs.values():
        entry = str(path)
        if entry in sys.path:
            sys.path.remove(entry)
        sys.path.insert(0, entry)
    for mod in [m for m in list(sys.modules) if m.split('.')[0] in names]:
        installed = getattr(sys.modules[mod], '__file__', '') or ''
        if _install_shadow(installed, names):
            del sys.modules[mod]
    return list(pkgs)


PINNED = _pin_to_source()
