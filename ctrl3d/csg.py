"""A tiny CSG scene graph with OpenSCAD and PythonSCAD back ends.

The same node tree can be serialized to OpenSCAD source or converted into
native PythonSCAD objects.  This keeps the mechanism core independent of the
CAD executable while still allowing one-click display inside PythonSCAD.
"""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple, Union

from .math3d import Transform, as_vec3

RGB = Tuple[float, float, float]


def _pyscad():
    try:
        import pythonscad  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - depends on the host app
        raise RuntimeError(
            "PythonSCAD is not available in this interpreter. Run the script "
            "inside PythonSCAD, or use Mechanism.save_scad()/to_scad() to "
            "produce an OpenSCAD file."
        ) from exc
    return pythonscad


def _fmt(value: float) -> str:
    if abs(value) < 5e-13:
        value = 0.0
    return f"{value:.10g}"


def _vector(values: Sequence[float]) -> str:
    return "[" + ", ".join(_fmt(float(value)) for value in values) + "]"


def parse_color(value: Union[str, Sequence[float]]) -> Tuple[RGB, float]:
    """Normalize a hex string or an RGB(A) sequence to ``((r, g, b), alpha)``."""
    if isinstance(value, str):
        text = value.strip().lstrip("#")
        if len(text) == 6:
            alpha = 1.0
        elif len(text) == 8:
            alpha = int(text[6:8], 16) / 255.0
            text = text[:6]
        else:
            raise ValueError("hex colors must have the form '#rrggbb' or '#rrggbbaa'")
        try:
            rgb = (
                int(text[0:2], 16) / 255.0,
                int(text[2:4], 16) / 255.0,
                int(text[4:6], 16) / 255.0,
            )
        except ValueError as exc:
            raise ValueError(f"invalid hex color {value!r}") from exc
        return rgb, alpha
    components = [float(item) for item in value]
    if len(components) not in (3, 4):
        raise ValueError("colors must be '#rrggbb' or an RGB(A) sequence")
    alpha = components[3] if len(components) == 4 else 1.0
    return tuple(components[:3]), alpha  # type: ignore[return-value]


class Solid:
    """Base class of the minimal scene graph."""

    def to_scad(self, indent: int = 0) -> str:
        raise NotImplementedError

    def to_pyscad(self):
        raise NotImplementedError

    @property
    def is_empty(self) -> bool:
        return False

    def show(self):
        _pyscad().show(self.to_pyscad())

    def export(self, path: str):
        _pyscad().export(self.to_pyscad(), path)


def _indent(text: str, spaces: int) -> str:
    prefix = " " * spaces
    return "\n".join(prefix + line if line else line for line in text.splitlines())


class Empty(Solid):
    @property
    def is_empty(self) -> bool:
        return True

    def to_scad(self, indent: int = 0) -> str:
        return " " * indent + "// empty"

    def to_pyscad(self):
        return None


class Cube(Solid):
    def __init__(self, size, center: bool = False) -> None:
        if isinstance(size, (int, float)):
            size = (float(size), float(size), float(size))
        self.size = as_vec3(size, "cube size")
        self.center = bool(center)

    def to_scad(self, indent: int = 0) -> str:
        return (
            " " * indent
            + f"cube(size={_vector(self.size)}, center={'true' if self.center else 'false'});"
        )

    def to_pyscad(self):
        ps = _pyscad()
        return ps.cube(list(self.size), center=self.center)


class Sphere(Solid):
    def __init__(self, r: float, segments: int = 48) -> None:
        if r <= 0:
            raise ValueError("sphere radius must be positive")
        self.r = float(r)
        self.segments = int(segments)

    def to_scad(self, indent: int = 0) -> str:
        return " " * indent + f"sphere(r={_fmt(self.r)}, $fn={self.segments});"

    def to_pyscad(self):
        return _pyscad().sphere(self.r, fn=self.segments)


class Cylinder(Solid):
    def __init__(
        self,
        h: float,
        r: float,
        r2: Optional[float] = None,
        center: bool = False,
        segments: int = 48,
    ) -> None:
        if h <= 0:
            raise ValueError("cylinder height must be positive")
        if r <= 0 or (r2 is not None and r2 <= 0):
            raise ValueError("cylinder radius must be positive")
        self.h = float(h)
        self.r = float(r)
        self.r2 = None if r2 is None else float(r2)
        self.center = bool(center)
        self.segments = int(segments)

    def to_scad(self, indent: int = 0) -> str:
        args = [
            f"h={_fmt(self.h)}",
            f"r1={_fmt(self.r)}",
            f"r2={_fmt(self.r2 if self.r2 is not None else self.r)}",
            f"center={'true' if self.center else 'false'}",
            f"$fn={self.segments}",
        ]
        return " " * indent + "cylinder(" + ", ".join(args) + ");"

    def to_pyscad(self):
        ps = _pyscad()
        return ps.cylinder(
            h=self.h,
            r1=self.r,
            r2=self.r2 if self.r2 is not None else self.r,
            center=self.center,
            fn=self.segments,
        )


class Union(Solid):
    def __init__(self, children: Iterable[Solid]) -> None:
        flattened: List[Solid] = []
        for child in children:
            if child is None or child.is_empty:
                continue
            if isinstance(child, Union):
                flattened.extend(child.children)
            else:
                flattened.append(child)
        self.children: Tuple[Solid, ...] = tuple(flattened)

    @property
    def is_empty(self) -> bool:
        return not self.children

    def to_scad(self, indent: int = 0) -> str:
        if not self.children:
            return " " * indent + "// empty"
        lines = [" " * indent + "union() {"]
        for child in self.children:
            lines.append(child.to_scad(indent + 2))
        lines.append(" " * indent + "}")
        return "\n".join(lines)

    def to_pyscad(self):
        if not self.children:
            return None
        ps = _pyscad()
        objects = [child.to_pyscad() for child in self.children]
        objects = [obj for obj in objects if obj is not None]
        if not objects:
            return None
        if len(objects) == 1:
            return objects[0]
        return ps.union(*objects)


class Difference(Solid):
    def __init__(self, base: Solid, subtract: Iterable[Solid] = ()) -> None:
        self.base = base
        self.subtract: Tuple[Solid, ...] = tuple(
            child for child in subtract if child is not None and not child.is_empty
        )

    @property
    def is_empty(self) -> bool:
        return self.base.is_empty

    def to_scad(self, indent: int = 0) -> str:
        lines = [" " * indent + "difference() {", self.base.to_scad(indent + 2)]
        for child in self.subtract:
            lines.append(child.to_scad(indent + 2))
        lines.append(" " * indent + "}")
        return "\n".join(lines)

    def to_pyscad(self):
        ps = _pyscad()
        base = self.base.to_pyscad()
        if not self.subtract:
            return base
        return ps.difference(base, *[child.to_pyscad() for child in self.subtract])


class Intersection(Solid):
    def __init__(self, children: Iterable[Solid]) -> None:
        self.children: Tuple[Solid, ...] = tuple(
            child for child in children if child is not None and not child.is_empty
        )

    @property
    def is_empty(self) -> bool:
        return not self.children

    def to_scad(self, indent: int = 0) -> str:
        if not self.children:
            return " " * indent + "// empty"
        lines = [" " * indent + "intersection() {"]
        for child in self.children:
            lines.append(child.to_scad(indent + 2))
        lines.append(" " * indent + "}")
        return "\n".join(lines)

    def to_pyscad(self):
        ps = _pyscad()
        objects = [child.to_pyscad() for child in self.children]
        objects = [obj for obj in objects if obj is not None]
        if not objects:
            return None
        return ps.intersection(*objects)


class Transformed(Solid):
    """Apply a rigid transform to a child."""

    def __init__(self, transform: Transform, child: Solid) -> None:
        self.transform = transform
        self.child = child

    @property
    def is_empty(self) -> bool:
        return self.child.is_empty

    def to_scad(self, indent: int = 0) -> str:
        matrix = self.transform.to_matrix4()
        rows = ", ".join(_vector(row) for row in matrix)
        lines = [
            " " * indent + f"multmatrix([{rows}]) {{",
            self.child.to_scad(indent + 2),
            " " * indent + "}",
        ]
        return "\n".join(lines)

    def to_pyscad(self):
        ps = _pyscad()
        return ps.multmatrix(
            self.child.to_pyscad(),
            [list(row) for row in self.transform.to_matrix4()],
        )


class Colored(Solid):
    def __init__(
        self,
        child: Solid,
        color: Union[str, Sequence[float]],
        alpha: Optional[float] = None,
    ) -> None:
        self.child = child
        self.rgb, parsed_alpha = parse_color(color)
        self.alpha = parsed_alpha if alpha is None else float(alpha)

    @property
    def is_empty(self) -> bool:
        return self.child.is_empty

    def to_scad(self, indent: int = 0) -> str:
        rgba = (*self.rgb, self.alpha)
        lines = [
            " " * indent + f"color({_vector(rgba)}) {{",
            self.child.to_scad(indent + 2),
            " " * indent + "}",
        ]
        return "\n".join(lines)

    def to_pyscad(self):
        ps = _pyscad()
        # PythonSCAD's color() accepts either a 4 component RGBA list or an
        # alpha keyword (the keyword form is not accepted by every release).
        return ps.color(
            self.child.to_pyscad(),
            [self.rgb[0], self.rgb[1], self.rgb[2], self.alpha],
        )


class Background(Solid):
    """Transparent reference geometry (``%`` modifier)."""

    def __init__(self, child: Solid) -> None:
        self.child = child

    @property
    def is_empty(self) -> bool:
        return self.child.is_empty

    def to_scad(self, indent: int = 0) -> str:
        lines = [
            " " * indent + "% {",
            self.child.to_scad(indent + 2),
            " " * indent + "}",
        ]
        return "\n".join(lines)

    def to_pyscad(self):
        return _pyscad().background(self.child.to_pyscad())
