"""Thin bridge to the PythonSCAD viewer / exporter.

Importing this module outside PythonSCAD is safe; the host module is only
imported when a display or export is actually requested.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from .csg import Solid


def is_available() -> bool:
    """True when the script is running inside PythonSCAD."""
    try:
        import pythonscad  # type: ignore[import-not-found]  # noqa: F401
    except ImportError:
        return False
    return True


def show_solid(solid: Optional[Solid]):
    if solid is None or solid.is_empty:
        raise ValueError("nothing to show: the mechanism produced no geometry")
    if not is_available():
        raise RuntimeError(
            "PythonSCAD is not available in this interpreter. Run the script "
            "inside PythonSCAD to display it, or call "
            "Mechanism.save_scad('mechanism.scad') instead."
        )
    import pythonscad  # type: ignore[import-not-found]

    return pythonscad.show(solid.to_pyscad())


def export_solid(solid: Optional[Solid], path: str) -> str:
    if solid is None or solid.is_empty:
        raise ValueError("nothing to export: the mechanism produced no geometry")
    if not is_available():
        raise RuntimeError(
            "Exporting STL/3MF/... needs PythonSCAD. Save OpenSCAD source "
            "with Mechanism.save_scad(path) instead."
        )
    import pythonscad  # type: ignore[import-not-found]

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    pythonscad.export(solid.to_pyscad(), str(target))
    return path
