"""Result objects produced by ctrl3d analyses.

Keeping these as small immutable records makes the numeric results easy to
print, compare and post-process (for example in design-space exploration).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class ClosureError:
    """Deviation of a joint from the motion its kinematic pair allows."""

    joint: str
    position: float
    angle: float  # degrees
    magnitude: float

    def __str__(self) -> str:
        return (
            f"{self.joint}: position={self.position:.3e}, "
            f"angle={self.angle:.3e} deg"
        )


@dataclass(frozen=True)
class Diagnostics:
    """Errors and warnings collected from a mechanism description."""

    errors: Tuple[str, ...] = ()
    warnings: Tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return not self.errors

    def __bool__(self) -> bool:
        return self.ok

    def format(self) -> str:
        lines = [f"error: {message}" for message in self.errors]
        lines += [f"warning: {message}" for message in self.warnings]
        return "\n".join(lines) if lines else "ok"


@dataclass(frozen=True)
class MobilityReport:
    """Degrees-of-freedom analysis of a mechanism."""

    name: str
    links: int
    joints: int
    fixed_links: int
    loops: int
    components: int
    space: str
    plane_normal: Optional[Tuple[float, float, float]]
    grubler: int
    screw: int
    actuated: int
    remaining: int
    warnings: Tuple[str, ...] = ()

    @property
    def consistent(self) -> bool:
        return self.grubler == self.screw

    def __str__(self) -> str:
        normal = (
            " normal=({:.3g}, {:.3g}, {:.3g})".format(*self.plane_normal)
            if self.plane_normal
            else ""
        )
        lines = [
            f"mechanism '{self.name}'",
            f"  links       : {self.links} (fixed: {self.fixed_links})",
            f"  joints      : {self.joints}",
            f"  loops       : {self.loops}",
            f"  space       : {self.space}{normal}",
            f"  Gruebler    : {self.grubler}",
            f"  screw (inst): {self.screw}",
            f"  inputs      : {self.actuated}",
            f"  remaining   : {self.remaining}",
        ]
        if not self.consistent:
            lines.append(
                "  note        : Gruebler and screw mobility differ "
                "(redundant/special geometry)"
            )
        for warning in self.warnings:
            lines.append(f"  warning     : {warning}")
        return "\n".join(lines)


@dataclass(frozen=True)
class GrashofResult:
    """Grashof classification of a planar four-bar linkage."""

    lengths: Dict[str, float]
    shortest: str
    longest: str
    is_grashof: bool
    change_point: bool
    kind: str

    @property
    def summary(self) -> str:
        if self.change_point:
            return "change-point (s + l == p + q)"
        if self.is_grashof:
            return f"Grashof {self.kind} (shortest link: '{self.shortest}')"
        return "non-Grashof; all links rock (triple rocker)"

    def __str__(self) -> str:
        lengths = ", ".join(
            f"{name}={value:.4g}" for name, value in self.lengths.items()
        )
        return f"{self.summary}; lengths: {lengths}"


@dataclass(frozen=True)
class SolveResult:
    """Outcome of a closure (inverse kinematics) solve."""

    converged: bool
    residual: float
    iterations: int
    restarts: int
    driver: Optional[str] = None
    driver_value: Optional[float] = None
    drivers: Dict[str, float] = field(default_factory=dict)
    values: Dict[str, float] = field(default_factory=dict)
    message: str = ""

    def __str__(self) -> str:
        if self.drivers:
            driver = ", " + ", ".join(
                f"{name}={value:g}" for name, value in self.drivers.items()
            )
        elif self.driver is not None and self.driver_value is not None:
            driver = f", {self.driver}={self.driver_value:g}"
        else:
            driver = ""
        state = "converged" if self.converged else "not converged"
        return (
            f"{state}: residual={self.residual:.3e}, "
            f"iterations={self.iterations}, restarts={self.restarts}{driver}"
        )
