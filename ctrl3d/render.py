"""Render a mechanism as CSG geometry (OpenSCAD / PythonSCAD)."""

from __future__ import annotations

from typing import List, Optional

from .csg import (
    Colored,
    Cube,
    Cylinder,
    Empty,
    Solid,
    Sphere,
    Transformed,
    Union,
)
from .joints import PrismaticJoint, RevoluteJoint
from .links import Link, LinkRole
from .math3d import Transform, Vec3, mat3_rotation_z_to, vdist, vsub
from .mechanism import Mechanism

ROLE_STYLE = {
    LinkRole.FIXED: ((0.56, 0.62, 0.68), 0.40),
    LinkRole.INPUT: ((0.85, 0.27, 0.24), 1.0),
    LinkRole.INTERMEDIATE: ((0.63, 0.66, 0.70), 1.0),
    LinkRole.OUTPUT: ((0.20, 0.47, 0.82), 1.0),
}

JOINT_STYLE = ((0.16, 0.16, 0.18), 1.0)
RAIL_STYLE = ((0.42, 0.44, 0.48), 1.0)

SEGMENTS = 40


def _segment(p: Vec3, q: Vec3, radius: float) -> Solid:
    direction = vsub(q, p)
    length = vdist(p, q)
    if length < 1e-12:
        return Empty()
    transform = Transform(mat3_rotation_z_to(direction), p)
    return Transformed(
        transform, Cylinder(length, radius, center=False, segments=SEGMENTS)
    )


def _sphere_at(point: Vec3, radius: float) -> Solid:
    return Transformed(
        Transform.translation(point), Sphere(radius, segments=SEGMENTS // 2)
    )


def _link_solid(link: Link, transform: Transform, default_radius: float) -> Solid:
    if link.shape == "none":
        return Empty()
    radius = link.radius if link.radius is not None else default_radius
    if link.role is LinkRole.FIXED:
        radius *= 1.45
    points = list(link.ports.values())
    parts: List[Solid] = []
    for a, b in zip(points, points[1:]):
        parts.append(_segment(a, b, radius))
    seen = set()
    for point in points:
        key = tuple(round(value, 12) for value in point)
        if key in seen:
            continue
        seen.add(key)
        parts.append(_sphere_at(point, radius))
    if not parts:
        return Empty()
    body = Transformed(transform, Union(parts))
    color, alpha = ROLE_STYLE[link.role]
    if link.color is not None:
        return Colored(body, link.color)
    return Colored(body, color, alpha=alpha)


def _revolute_solid(joint: RevoluteJoint, frame: Transform, default_radius: float) -> Solid:
    pin_radius = default_radius * 1.12
    pin_length = default_radius * 3.1
    pin_frame = frame @ Transform(mat3_rotation_z_to(joint.axis))
    pin = Transformed(
        pin_frame,
        Cylinder(pin_length, pin_radius, center=True, segments=SEGMENTS),
    )
    color, alpha = JOINT_STYLE
    return Colored(pin, color, alpha=alpha)


def _prismatic_solid(
    joint: PrismaticJoint, frame: Transform, default_radius: float
) -> Solid:
    if joint.limits is not None:
        low, high = joint.limits
        low -= default_radius * 2.0
        high += default_radius * 2.0
    else:
        low = min(0.0, joint.value) - default_radius * 2.5
        high = max(0.0, joint.value) + default_radius * 2.5
    if high - low < default_radius * 4.0:
        centre = (low + high) / 2.0
        low = centre - default_radius * 2.0
        high = centre + default_radius * 2.0
    rail_start = frame.apply((joint.axis[0] * low, joint.axis[1] * low, joint.axis[2] * low))
    rail_end = frame.apply((joint.axis[0] * high, joint.axis[1] * high, joint.axis[2] * high))
    rail = _segment(rail_start, rail_end, default_radius * 0.42)
    slider_center = frame.apply(
        (
            joint.axis[0] * joint.value,
            joint.axis[1] * joint.value,
            joint.axis[2] * joint.value,
        )
    )
    slider_frame = (
        Transform(
            mat3_rotation_z_to(joint.axis),
            slider_center,
        )
    )
    size = default_radius * 2.1
    slider = Transformed(
        slider_frame,
        Cube((size, size, size * 1.6), center=True),
    )
    rail_color, rail_alpha = RAIL_STYLE
    joint_color, joint_alpha = JOINT_STYLE
    return Union(
        [
            Colored(rail, rail_color, alpha=rail_alpha),
            Colored(slider, joint_color, alpha=joint_alpha),
        ]
    )


def mechanism_solid(mechanism: Mechanism) -> Solid:
    """Build the CSG tree of the mechanism in its current configuration."""
    poses = mechanism.poses()
    default_radius = mechanism.default_radius
    parts: List[Solid] = []
    for link in mechanism.links:
        part = _link_solid(link, poses[link.name], default_radius)
        if not part.is_empty:
            parts.append(part)
    for joint in mechanism.joints:
        frame = poses[joint.port_a.link.name] @ Transform.translation(
            joint.port_a.link.port(joint.port_a.port)
        )
        if isinstance(joint, RevoluteJoint):
            parts.append(_revolute_solid(joint, frame, default_radius))
        elif isinstance(joint, PrismaticJoint):
            parts.append(_prismatic_solid(joint, frame, default_radius))
    return Union(parts)


def link_solid(
    mechanism: Mechanism, link: Link, default_radius: Optional[float] = None
) -> Solid:
    """Geometry of a single link, useful for detail views and exports."""
    poses = mechanism.poses()
    return _link_solid(
        link,
        poses[link.name],
        default_radius if default_radius is not None else mechanism.default_radius,
    )
