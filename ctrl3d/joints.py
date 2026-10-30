"""Kinematic pairs ("対偶") used by ctrl3d.

The module implements the two elementary one-degree-of-freedom pairs used in
most planar and spatial mechanisms:

``RevoluteJoint``  回転対偶 / R pair: one rotation about an axis.
``PrismaticJoint`` 直進対偶 / P pair: one translation along an axis.

A joint connects one port of link A to one port of link B.  Its axis is
expressed in the local frame of link A's port.  The joint value is the angle
in degrees for an R pair and the displacement for a P pair.
"""

from __future__ import annotations

import math
from typing import ClassVar, Optional, Sequence, Tuple, Union

from .links import Link, PortRef
from .math3d import Transform, as_vec3, vscale, vunit

Limits = Tuple[float, float]


class Joint:
    """Base class for a one-DOF kinematic pair."""

    kind: ClassVar[str] = "joint"
    units: ClassVar[str] = ""
    default_axis: ClassVar[Tuple[float, float, float]] = (0.0, 0.0, 1.0)

    def __init__(
        self,
        name: str,
        a: Union[PortRef, Tuple[Link, str]],
        b: Union[PortRef, Tuple[Link, str]],
        *,
        axis: Optional[Sequence[float]] = None,
        value: float = 0.0,
        limits: Optional[Limits] = None,
        actuated: bool = False,
    ) -> None:
        if not isinstance(name, str) or not name.strip():
            raise ValueError("joint name must be a non-empty string")
        self.name = name
        self.port_a = a if isinstance(a, PortRef) else PortRef(a[0], a[1])
        self.port_b = b if isinstance(b, PortRef) else PortRef(b[0], b[1])

        if self.port_a.link is self.port_b.link:
            raise ValueError(f"joint '{name}' must connect two different links")
        self.port_a.link.port(self.port_a.port)
        self.port_b.link.port(self.port_b.port)

        if axis is None:
            axis = self.default_axis
        self.axis = vunit(as_vec3(axis, f"axis of joint '{name}'"), f"axis of joint '{name}'")
        self.actuated = bool(actuated)
        self.limits = self._normalize_limits(limits)
        self._value = 0.0
        self.set_value(value)

    @staticmethod
    def _normalize_limits(limits: Optional[Limits]) -> Optional[Limits]:
        if limits is None:
            return None
        try:
            low, high = (float(limits[0]), float(limits[1]))
        except (TypeError, IndexError, ValueError) as exc:
            raise TypeError("limits must be a (min, max) pair") from exc
        if not (math.isfinite(low) and math.isfinite(high)):
            raise ValueError("joint limits must be finite")
        if low > high:
            raise ValueError("joint limit minimum must not exceed maximum")
        return (low, high)

    # -- value --------------------------------------------------------------
    @property
    def value(self) -> float:
        return self._value

    @value.setter
    def value(self, new_value: float) -> None:
        self.set_value(new_value)

    def set_value(self, new_value: float) -> float:
        value = float(new_value)
        if not math.isfinite(value):
            raise ValueError(f"joint '{self.name}' value must be finite")
        self._value = value
        return value

    @property
    def in_limits(self) -> bool:
        if self.limits is None:
            return True
        return self.limits[0] - 1e-12 <= self._value <= self.limits[1] + 1e-12

    def clamp(self) -> float:
        if self.limits is not None:
            self._value = min(max(self._value, self.limits[0]), self.limits[1])
        return self._value

    # -- geometry -----------------------------------------------------------
    @property
    def motion(self) -> Transform:
        """Transform from port A's frame to port B's frame at this value."""
        raise NotImplementedError

    @property
    def dof(self) -> int:
        return 1

    @property
    def links(self) -> Tuple[Link, Link]:
        return (self.port_a.link, self.port_b.link)

    def connects(self, link: Link) -> bool:
        return link is self.port_a.link or link is self.port_b.link

    def other_link(self, link: Link) -> Link:
        if link is self.port_a.link:
            return self.port_b.link
        if link is self.port_b.link:
            return self.port_a.link
        raise ValueError(f"joint '{self.name}' does not connect link '{link.name}'")

    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}(name={self.name!r}, "
            f"{self.port_a} <-> {self.port_b}, axis={self.axis!r}, "
            f"value={self.value:g}{self.units})"
        )


class RevoluteJoint(Joint):
    """回転対偶 / revolute pair: rotation about a fixed axis."""

    kind: ClassVar[str] = "revolute"
    units: ClassVar[str] = "deg"
    default_axis: ClassVar[Tuple[float, float, float]] = (0.0, 0.0, 1.0)

    @property
    def motion(self) -> Transform:
        return Transform.rotation(self.axis, self.value)

    @property
    def angle(self) -> float:
        """Rotation angle in degrees."""
        return self.value

    @angle.setter
    def angle(self, value: float) -> None:
        self.set_value(value)

    @property
    def angle_rad(self) -> float:
        return math.radians(self.value)


class PrismaticJoint(Joint):
    """直進対偶 / prismatic pair: translation along a fixed axis."""

    kind: ClassVar[str] = "prismatic"
    units: ClassVar[str] = "length"
    default_axis: ClassVar[Tuple[float, float, float]] = (1.0, 0.0, 0.0)

    @property
    def motion(self) -> Transform:
        return Transform.translation(vscale(self.axis, self.value))

    @property
    def displacement(self) -> float:
        return self.value

    @displacement.setter
    def displacement(self, value: float) -> None:
        self.set_value(value)
