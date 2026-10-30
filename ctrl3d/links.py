"""Link ("節") descriptions for planar and spatial mechanisms.

A link is a rigid body with named *ports*.  Joints connect two ports; the
kinematics of the mechanism are solved from those connections.  The four
design roles used by the module are:

``fixed``        固定節 / ground link (the frame)
``input``        入力節 / driving link
``intermediate`` 中間節 / coupler, rocker or other transmission link
``output``       出力節 / driven link

Links may be described either by a ``length`` (ports ``A`` at the origin and
``B`` at ``(length, 0, 0)``) or by explicit port coordinates.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Iterator, Mapping, Optional, Sequence, Union

from .math3d import Vec3, as_vec3, vdist

Color = Union[str, Sequence[float]]


class LinkRole(str, Enum):
    """Design role of a link."""

    FIXED = "fixed"
    INPUT = "input"
    INTERMEDIATE = "intermediate"
    OUTPUT = "output"


@dataclass(frozen=True)
class PortRef:
    """Reference to one named port of a link."""

    link: "Link"
    port: str

    def __str__(self) -> str:
        return f"{self.link.name}.{self.port}"


class Link:
    """A rigid link with named attachment ports."""

    def __init__(
        self,
        name: str,
        ports: Optional[Mapping[str, Sequence[float]]] = None,
        *,
        length: Optional[float] = None,
        role: Union[LinkRole, str] = LinkRole.INTERMEDIATE,
        radius: Optional[float] = None,
        color: Optional[Color] = None,
        shape: str = "bar",
    ) -> None:
        if not isinstance(name, str) or not name.strip():
            raise ValueError("link name must be a non-empty string")
        self.name = name
        try:
            self.role = LinkRole(role)
        except ValueError as exc:
            raise ValueError(
                f"unknown link role {role!r}; use one of "
                f"{', '.join(item.value for item in LinkRole)}"
            ) from exc

        if ports is None:
            if length is None:
                raise ValueError(
                    f"link '{name}' needs either ports=... or length=..."
                )
            if length <= 0:
                raise ValueError(f"link '{name}' length must be positive")
            ports = {"A": (0.0, 0.0, 0.0), "B": (float(length), 0.0, 0.0)}
        elif length is not None:
            raise ValueError(
                f"link '{name}': pass either ports=... or length=..., not both"
            )

        if not ports:
            raise ValueError(f"link '{name}' must define at least one port")
        normalized: Dict[str, Vec3] = {}
        for port_name, position in ports.items():
            if not isinstance(port_name, str) or not port_name:
                raise ValueError(f"link '{name}': port names must be non-empty strings")
            normalized[str(port_name)] = as_vec3(
                position, f"port '{port_name}' of link '{name}'"
            )
        self.ports: Dict[str, Vec3] = normalized

        if radius is not None and radius <= 0:
            raise ValueError(f"link '{name}' radius must be positive")
        if shape not in ("bar", "none"):
            raise ValueError(f"link '{name}': shape must be 'bar' or 'none'")
        self.radius = None if radius is None else float(radius)
        self.color = color
        self.shape = shape

    # -- ports --------------------------------------------------------------
    def port(self, name: str) -> Vec3:
        try:
            return self.ports[name]
        except KeyError as exc:
            available = ", ".join(self.ports) or "none"
            raise KeyError(
                f"link '{self.name}' has no port '{name}' (available: {available})"
            ) from exc

    def __getitem__(self, port_name: str) -> PortRef:
        self.port(port_name)
        return PortRef(self, port_name)

    def __iter__(self) -> Iterator[str]:
        return iter(self.ports)

    def port_names(self) -> tuple:
        return tuple(self.ports)

    # -- geometry -----------------------------------------------------------
    @property
    def extent(self) -> float:
        """Largest distance between two ports of the link."""
        points = list(self.ports.values())
        return max(
            (vdist(a, b) for i, a in enumerate(points) for b in points[i + 1 :]),
            default=0.0,
        )

    @property
    def length(self) -> float:
        """Distance between the first and last declared port."""
        points = list(self.ports.values())
        if len(points) < 2:
            return 0.0
        return vdist(points[0], points[-1])

    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}(name={self.name!r}, role={self.role.value!r}, "
            f"ports={list(self.ports)!r})"
        )


class FixedLink(Link):
    """固定節: link that is rigidly attached to the world frame."""

    def __init__(self, name: str, ports=None, **kwargs) -> None:
        kwargs.pop("role", None)
        super().__init__(name, ports, role=LinkRole.FIXED, **kwargs)


class InputLink(Link):
    """入力節: link that receives the drive motion."""

    def __init__(self, name: str, ports=None, **kwargs) -> None:
        kwargs.pop("role", None)
        super().__init__(name, ports, role=LinkRole.INPUT, **kwargs)


class IntermediateLink(Link):
    """中間節: coupler / rocker / transmission link."""

    def __init__(self, name: str, ports=None, **kwargs) -> None:
        kwargs.pop("role", None)
        super().__init__(name, ports, role=LinkRole.INTERMEDIATE, **kwargs)


class OutputLink(Link):
    """出力節: link that delivers the output motion."""

    def __init__(self, name: str, ports=None, **kwargs) -> None:
        kwargs.pop("role", None)
        super().__init__(name, ports, role=LinkRole.OUTPUT, **kwargs)


def ports_along_line(
    length: float, count: int = 2, start: float = 0.0
) -> Dict[str, Vec3]:
    """Convenience helper creating evenly spaced ports on the X axis."""
    if count < 2:
        raise ValueError("count must be at least 2")
    if length <= 0:
        raise ValueError("length must be positive")
    step = length / (count - 1)
    names = [chr(ord("A") + index) for index in range(count)]
    return {name: (start + index * step, 0.0, 0.0) for index, name in enumerate(names)}
