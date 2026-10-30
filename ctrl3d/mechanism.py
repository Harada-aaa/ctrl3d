"""Mechanism description, kinematics and degree-of-freedom analysis.

A :class:`Mechanism` owns links and joints.  From the connection graph it can

* assemble the current pose of every link (spanning-tree kinematics),
* report the remaining loop-closure error,
* solve the passive joint coordinates for a prescribed input,
* compute the degree of freedom with the Gruebler/Kutzbach criterion and
  with an instantaneous screw-based criterion, and
* emit OpenSCAD geometry or display itself inside PythonSCAD.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Union

from .analysis import (
    ClosureError,
    Diagnostics,
    GrashofResult,
    MobilityReport,
    SolveResult,
)
from .joints import Joint, PrismaticJoint, RevoluteJoint
from .links import (
    FixedLink,
    InputLink,
    IntermediateLink,
    Link,
    LinkRole,
    OutputLink,
    PortRef,
)
from .math3d import (
    Transform,
    Vec3,
    max_abs,
    rank,
    rotation_angle_deg,
    rotation_log,
    solve_linear,
    vcross,
    vdist,
    vdot,
    vnorm,
    vparallel,
    vscale,
    vunit,
)

Endpoint = Union[PortRef, Tuple[Link, str], str, Link]


class _UnionFind:
    def __init__(self, items: Iterable[Link]) -> None:
        self.parent = {item: item for item in items}

    def find(self, item: Link) -> Link:
        root = item
        while self.parent[root] is not root:
            root = self.parent[root]
        while self.parent[item] is not item:
            self.parent[item], item = root, self.parent[item]
        return root

    def union(self, a: Link, b: Link) -> bool:
        ra, rb = self.find(a), self.find(b)
        if ra is rb:
            return False
        self.parent[rb] = ra
        return True


class Mechanism:
    """A collection of links and kinematic pairs."""

    def __init__(self, name: str = "mechanism") -> None:
        if not isinstance(name, str) or not name.strip():
            raise ValueError("mechanism name must be a non-empty string")
        self.name = name
        self._links: Dict[str, Link] = {}
        self._joints: Dict[str, Joint] = {}
        self._joint_counter = 0

    # ------------------------------------------------------------------
    # construction
    # ------------------------------------------------------------------
    def add(self, link: Link) -> Link:
        if not isinstance(link, Link):
            raise TypeError("add() expects a Link instance")
        if link.name in self._links:
            raise ValueError(f"duplicate link name '{link.name}'")
        self._links[link.name] = link
        return link

    def _two_port_definition(self, ports, length, spacing):
        if spacing is not None:
            if ports is not None:
                raise ValueError("pass either ports=... or spacing=..., not both")
            if length is not None:
                raise ValueError("pass either length=... or spacing=..., not both")
            if spacing <= 0:
                raise ValueError("spacing must be positive")
            return {"A": (0.0, 0.0, 0.0), "B": (float(spacing), 0.0, 0.0)}, None
        return ports, length

    def fixed(self, name: str, ports=None, *, spacing=None, **kwargs) -> FixedLink:
        ports, length = self._two_port_definition(ports, kwargs.pop("length", None), spacing)
        return self.add(FixedLink(name, ports, length=length, **kwargs))

    def input(self, name: str, ports=None, *, spacing=None, **kwargs) -> InputLink:
        ports, length = self._two_port_definition(ports, kwargs.pop("length", None), spacing)
        return self.add(InputLink(name, ports, length=length, **kwargs))

    def intermediate(
        self, name: str, ports=None, *, spacing=None, **kwargs
    ) -> IntermediateLink:
        ports, length = self._two_port_definition(ports, kwargs.pop("length", None), spacing)
        return self.add(IntermediateLink(name, ports, length=length, **kwargs))

    def output(self, name: str, ports=None, *, spacing=None, **kwargs) -> OutputLink:
        ports, length = self._two_port_definition(ports, kwargs.pop("length", None), spacing)
        return self.add(OutputLink(name, ports, length=length, **kwargs))

    def add_joint(self, joint: Joint) -> Joint:
        if not isinstance(joint, Joint):
            raise TypeError("add_joint() expects a Joint instance")
        if joint.name in self._joints:
            raise ValueError(f"duplicate joint name '{joint.name}'")
        for ref in (joint.port_a, joint.port_b):
            registered = self._links.get(ref.link.name)
            if registered is not ref.link:
                raise ValueError(
                    f"joint '{joint.name}' uses link '{ref.link.name}' "
                    "which is not part of this mechanism"
                )
            ref.link.port(ref.port)
        for existing in self._joints.values():
            if (
                existing.port_a.link is joint.port_a.link
                and existing.port_b.link is joint.port_b.link
                and existing.port_a.port == joint.port_a.port
                and existing.port_b.port == joint.port_b.port
            ):
                raise ValueError(
                    f"joint '{joint.name}' duplicates the connection of "
                    f"joint '{existing.name}'"
                )
        self._joints[joint.name] = joint
        return joint

    def _next_joint_name(self) -> str:
        while True:
            self._joint_counter += 1
            name = f"J{self._joint_counter}"
            if name not in self._joints:
                return name

    def _resolve_endpoint(self, endpoint: Endpoint) -> PortRef:
        if isinstance(endpoint, PortRef):
            return endpoint
        if isinstance(endpoint, Link):
            if len(endpoint.ports) == 1:
                return PortRef(endpoint, next(iter(endpoint.ports)))
            raise ValueError(
                f"link '{endpoint.name}' has several ports; "
                f"select one with '{endpoint.name}[\"port\"]'"
            )
        if isinstance(endpoint, str):
            if "." not in endpoint:
                raise ValueError(
                    f"endpoint string '{endpoint}' must have the form 'link.port'"
                )
            link_name, port_name = endpoint.rsplit(".", 1)
            link = self._links.get(link_name)
            if link is None:
                raise KeyError(f"unknown link '{link_name}'")
            return PortRef(link, port_name)
        if isinstance(endpoint, tuple) and len(endpoint) == 2:
            link, port = endpoint
            if isinstance(link, str):
                found = self._links.get(link)
                if found is None:
                    raise KeyError(f"unknown link '{link}'")
                link = found
            if not isinstance(link, Link) or not isinstance(port, str):
                raise TypeError("endpoint tuple must be (Link, port_name)")
            return PortRef(link, port)
        raise TypeError(
            "endpoint must be a PortRef such as link['A'], a 'link.port' string, "
            "a (link, port) tuple, or a single-port Link"
        )

    def _two_endpoints(self, a: Endpoint, b: Optional[Endpoint]) -> Tuple[PortRef, PortRef]:
        if b is None:
            if isinstance(a, tuple) and len(a) == 2 and all(
                isinstance(item, (PortRef, Link, str, tuple)) for item in a
            ):
                try:
                    return self._resolve_endpoint(a[0]), self._resolve_endpoint(a[1])
                except (TypeError, ValueError, KeyError):
                    pass
            raise ValueError("a joint needs two endpoints")
        return self._resolve_endpoint(a), self._resolve_endpoint(b)

    def _auto_actuated(self, a: Link, b: Link) -> bool:
        roles = {a.role, b.role}
        return LinkRole.INPUT in roles and LinkRole.FIXED in roles

    def revolute(
        self,
        a: Endpoint,
        b: Optional[Endpoint] = None,
        *,
        axis: Sequence[float] = (0.0, 0.0, 1.0),
        name: Optional[str] = None,
        value: float = 0.0,
        limits: Optional[Tuple[float, float]] = None,
        actuated: Optional[bool] = None,
    ) -> RevoluteJoint:
        ref_a, ref_b = self._two_endpoints(a, b)
        if actuated is None:
            actuated = self._auto_actuated(ref_a.link, ref_b.link)
        joint = RevoluteJoint(
            name or self._next_joint_name(),
            ref_a,
            ref_b,
            axis=axis,
            value=value,
            limits=limits,
            actuated=actuated,
        )
        return self.add_joint(joint)  # type: ignore[return-value]

    def prismatic(
        self,
        a: Endpoint,
        b: Optional[Endpoint] = None,
        *,
        axis: Sequence[float] = (1.0, 0.0, 0.0),
        name: Optional[str] = None,
        value: float = 0.0,
        limits: Optional[Tuple[float, float]] = None,
        actuated: Optional[bool] = None,
    ) -> PrismaticJoint:
        ref_a, ref_b = self._two_endpoints(a, b)
        if actuated is None:
            actuated = self._auto_actuated(ref_a.link, ref_b.link)
        joint = PrismaticJoint(
            name or self._next_joint_name(),
            ref_a,
            ref_b,
            axis=axis,
            value=value,
            limits=limits,
            actuated=actuated,
        )
        return self.add_joint(joint)  # type: ignore[return-value]

    # ------------------------------------------------------------------
    # accessors
    # ------------------------------------------------------------------
    @property
    def links(self) -> Tuple[Link, ...]:
        return tuple(self._links.values())

    @property
    def joints(self) -> Tuple[Joint, ...]:
        return tuple(self._joints.values())

    def link(self, name: str) -> Link:
        try:
            return self._links[name]
        except KeyError as exc:
            raise KeyError(f"unknown link '{name}'") from exc

    def joint(self, name: str) -> Joint:
        if isinstance(name, Joint):
            if name not in self._joints.values():
                raise ValueError(f"joint '{name.name}' is not part of this mechanism")
            return name
        try:
            return self._joints[str(name)]
        except KeyError as exc:
            raise KeyError(f"unknown joint '{name}'") from exc

    @property
    def link_count(self) -> int:
        return len(self._links)

    @property
    def joint_count(self) -> int:
        return len(self._joints)

    @property
    def fixed_links(self) -> Tuple[Link, ...]:
        return tuple(link for link in self._links.values() if link.role is LinkRole.FIXED)

    @property
    def input_links(self) -> Tuple[Link, ...]:
        return tuple(link for link in self._links.values() if link.role is LinkRole.INPUT)

    @property
    def output_links(self) -> Tuple[Link, ...]:
        return tuple(link for link in self._links.values() if link.role is LinkRole.OUTPUT)

    @property
    def actuated_joints(self) -> Tuple[Joint, ...]:
        return tuple(joint for joint in self._joints.values() if joint.actuated)

    @property
    def input_count(self) -> int:
        return len(self.actuated_joints)

    @property
    def extent(self) -> float:
        return max((link.extent for link in self.links), default=0.0)

    @property
    def characteristic_length(self) -> float:
        extent = self.extent
        return extent if extent > 1e-9 else 1.0

    @property
    def default_radius(self) -> float:
        return max(self.characteristic_length * 0.06, 1e-6)

    def state(self) -> Dict[str, float]:
        return {joint.name: joint.value for joint in self.joints}

    def set_state(self, values: Dict[Union[str, Joint], float]) -> "Mechanism":
        for key, value in values.items():
            self.joint(key).set_value(value)
        return self

    # ------------------------------------------------------------------
    # graph helpers
    # ------------------------------------------------------------------
    def _is_locked(self, joint: Joint) -> bool:
        return all(link.role is LinkRole.FIXED for link in joint.links)

    def _components(self) -> List[List[Link]]:
        uf = _UnionFind(self.links)
        for joint in self.joints:
            uf.union(joint.port_a.link, joint.port_b.link)
        groups: Dict[Link, List[Link]] = {}
        for link in self.links:
            groups.setdefault(uf.find(link), []).append(link)
        return list(groups.values())

    @property
    def component_count(self) -> int:
        return len(self._components())

    @property
    def loop_count(self) -> int:
        if not self._links:
            return 0
        return self.joint_count - self.link_count + self.component_count

    @property
    def _fixed_components(self) -> List[List[Link]]:
        result = []
        for component in self._components():
            if any(link.role is LinkRole.FIXED for link in component):
                result.append(component)
        return result

    # ------------------------------------------------------------------
    # kinematics
    # ------------------------------------------------------------------
    def poses(self) -> Dict[str, Transform]:
        """Pose of every link in the world frame at the current joint values."""
        poses: Dict[str, Transform] = {
            link.name: Transform.identity() for link in self.fixed_links
        }
        pending = list(self.joints)
        while pending:
            remaining: List[Joint] = []
            for joint in pending:
                a, b = joint.port_a, joint.port_b
                placed_a = a.link.name in poses
                placed_b = b.link.name in poses
                if placed_a and not placed_b:
                    poses[b.link.name] = self._pose_forward(poses[a.link.name], joint)
                elif placed_b and not placed_a:
                    poses[a.link.name] = self._pose_backward(poses[b.link.name], joint)
                elif not placed_a and not placed_b:
                    remaining.append(joint)
            if len(remaining) == len(pending):
                floating = next(
                    (link for link in self._links.values() if link.name not in poses),
                    None,
                )
                if floating is None:
                    break
                poses[floating.name] = Transform.identity()
            pending = remaining
        # every declared link receives a pose (isolated links are floating)
        for link in self.links:
            poses.setdefault(link.name, Transform.identity())
        return poses

    def _pose_forward(self, pose_a: Transform, joint: Joint) -> Transform:
        port_a = joint.port_a.link.port(joint.port_a.port)
        port_b = joint.port_b.link.port(joint.port_b.port)
        frame_a = pose_a @ Transform.translation(port_a)
        frame_b = frame_a @ joint.motion
        return frame_b @ Transform.translation(vscale(port_b, -1.0))

    def _pose_backward(self, pose_b: Transform, joint: Joint) -> Transform:
        port_a = joint.port_a.link.port(joint.port_a.port)
        port_b = joint.port_b.link.port(joint.port_b.port)
        frame_b = pose_b @ Transform.translation(port_b)
        frame_a = frame_b @ joint.motion.inverse()
        return frame_a @ Transform.translation(vscale(port_a, -1.0))

    def joint_frame(self, joint: Union[str, Joint]) -> Transform:
        """World frame of a joint's A-side port (joint axis origin)."""
        joint = self.joint(joint)
        poses = self.poses()
        port = joint.port_a.link.port(joint.port_a.port)
        return poses[joint.port_a.link.name] @ Transform.translation(port)

    def joint_axis_world(self, joint: Union[str, Joint]) -> Vec3:
        joint = self.joint(joint)
        return self.joint_frame(joint).apply_vector(joint.axis)

    def point(self, link: Union[str, Link], port: str) -> Vec3:
        link_obj = self.link(link) if isinstance(link, str) else link
        poses = self.poses()
        return poses[link_obj.name].apply(link_obj.port(port))

    def closure_errors(self) -> Dict[str, ClosureError]:
        """Loop-closure residual of every joint at the current state."""
        poses = self.poses()
        scale = self.characteristic_length
        errors: Dict[str, ClosureError] = {}
        for joint in self.joints:
            frame_a = poses[joint.port_a.link.name] @ Transform.translation(
                joint.port_a.link.port(joint.port_a.port)
            )
            frame_b = poses[joint.port_b.link.name] @ Transform.translation(
                joint.port_b.link.port(joint.port_b.port)
            )
            relative = frame_a.inverse() @ frame_b
            delta = joint.motion.inverse() @ relative
            position = vnorm(delta.t)
            angle = rotation_angle_deg(delta.R)
            errors[joint.name] = ClosureError(
                joint=joint.name,
                position=position,
                angle=angle,
                magnitude=position + math.radians(angle) * scale,
            )
        return errors

    def max_closure_error(self) -> float:
        return max((error.magnitude for error in self.closure_errors().values()), default=0.0)

    # ------------------------------------------------------------------
    # degree of freedom
    # ------------------------------------------------------------------
    def planarity(self) -> Tuple[bool, Optional[Vec3]]:
        """Detect whether the mechanism moves in a plane.

        Returns ``(is_planar, plane_normal)``.
        """
        joints = self.joints
        if not joints:
            return True, (0.0, 0.0, 1.0)
        poses = self.poses()
        revolutes = [joint for joint in joints if isinstance(joint, RevoluteJoint)]
        prismatics = [joint for joint in joints if isinstance(joint, PrismaticJoint)]
        r_axes = [self._world_axis(joint, poses) for joint in revolutes]
        p_axes = [self._world_axis(joint, poses) for joint in prismatics]
        if r_axes:
            normal = r_axes[0]
            if all(vparallel(normal, axis) for axis in r_axes[1:]):
                if all(abs(vdot(normal, axis)) <= 1e-7 for axis in p_axes):
                    return True, vunit(normal)
            return False, None
        if not p_axes:
            return True, (0.0, 0.0, 1.0)
        direction_rank = rank([list(axis) for axis in p_axes], tol=1e-8)
        if direction_rank <= 1:
            reference = p_axes[0]
            helper = (1.0, 0.0, 0.0) if abs(reference[0]) < 0.9 else (0.0, 1.0, 0.0)
            return True, vunit(vcross(reference, helper))
        if direction_rank == 2:
            first, second = self._independent_pair(p_axes)
            return True, vunit(vcross(first, second))
        return False, None

    @staticmethod
    def _independent_pair(vectors: List[Vec3]) -> Tuple[Vec3, Vec3]:
        first = vectors[0]
        for candidate in vectors[1:]:
            if vnorm(vcross(first, candidate)) > 1e-6:
                return first, candidate
        return first, vectors[1]

    @staticmethod
    def _world_axis(joint: Joint, poses: Dict[str, Transform]) -> Vec3:
        pose = poses[joint.port_a.link.name]
        return vunit(pose.apply_vector(joint.axis))

    def dof(self, space: str = "auto") -> int:
        """Degree of freedom by the Gruebler/Kutzbach criterion.

        ``space`` is ``"auto"`` (detect planar motion), ``"planar"``,
        ``"spatial"`` or ``"screw"`` (instantaneous screw mobility).
        """
        if space == "screw":
            return self.screw_dof()
        if not self.links:
            return 0
        if space == "auto":
            planar, _ = self.planarity()
        elif space == "planar":
            planar = True
        elif space == "spatial":
            planar = False
        else:
            raise ValueError("space must be 'auto', 'planar', 'spatial' or 'screw'")
        lambda_ = 3 if planar else 6
        fixed = len(self.fixed_links)
        moving = self.link_count - fixed
        constraints = sum(
            lambda_ - joint.dof
            for joint in self.joints
            if not self._is_locked(joint)
        )
        return lambda_ * moving - constraints

    def screw_dof(self) -> int:
        """Instantaneous mobility from the rank of the loop screw matrix.

        This recognizes redundant constraints and special (overconstrained)
        geometries such as Bennett's linkage far better than the Gruebler
        count.  Floating components contribute their global rigid-body DOF.
        """
        if not self.links:
            return 0
        planar, _ = self.planarity()
        lambda_ = 3 if planar else 6
        poses = self.poses()
        loops = self._loop_sequences()
        movable = [joint for joint in self.joints if not self._is_locked(joint)]
        column = {joint: index for index, joint in enumerate(movable)}
        rows: List[List[float]] = []
        for steps in loops:
            block = [[0.0] * len(movable) for _ in range(6)]
            for joint, sign in steps:
                index = column.get(joint)
                if index is None:
                    continue
                screw = self._screw(joint, sign, poses)
                for row in range(6):
                    # A tree edge shared by both halves of a loop path is
                    # traversed twice and its signed screws cancel here.
                    block[row][index] += screw[row]
            rows.extend(block)
        rank_value = rank(rows, tol=1e-8)
        mobility = len(movable) - rank_value
        for component in self._components():
            if not any(link.role is LinkRole.FIXED for link in component):
                mobility += lambda_
        return mobility

    @staticmethod
    def _screw(joint: Joint, sign: float, poses: Dict[str, Transform]) -> List[float]:
        frame = poses[joint.port_a.link.name] @ Transform.translation(
            joint.port_a.link.port(joint.port_a.port)
        )
        axis = frame.apply_vector(joint.axis)
        if isinstance(joint, RevoluteJoint):
            omega = vscale(axis, sign)
            linear = vscale(vcross(frame.t, axis), sign)
            return [omega[0], omega[1], omega[2], linear[0], linear[1], linear[2]]
        direction = vscale(axis, sign)
        return [0.0, 0.0, 0.0, direction[0], direction[1], direction[2]]

    def mobility_report(self) -> MobilityReport:
        planar, normal = self.planarity()
        space = "planar" if planar else "spatial"
        grubler = self.dof(space)
        screw = self.screw_dof()
        actuated = self.input_count
        remaining = grubler - actuated
        warnings: List[str] = []
        if grubler < 0:
            warnings.append(
                "Gruebler mobility is negative: the mechanism is overconstrained "
                "or needs special link dimensions"
            )
        if screw != grubler:
            if screw > grubler:
                warnings.append(
                    f"instantaneous screw mobility ({screw}) exceeds Gruebler "
                    f"mobility ({grubler}); the current state is likely a singular "
                    "configuration"
                )
            else:
                warnings.append(
                    f"instantaneous screw mobility ({screw}) is below Gruebler "
                    f"mobility ({grubler}); the mechanism has redundant constraints "
                    "or special (overconstrained) geometry"
                )
        if not self.fixed_links:
            warnings.append(
                "no fixed link: global rigid-body degrees of freedom are included"
            )
        if remaining > 0:
            warnings.append(
                f"{remaining} degree(s) of freedom remain; "
                f"{actuated} joint(s) are actuated"
            )
        elif remaining < 0:
            warnings.append(
                f"{-remaining} more joint(s) are actuated than the mechanism's "
                "degree of freedom (over-actuation)"
            )
        return MobilityReport(
            name=self.name,
            links=self.link_count,
            joints=self.joint_count,
            fixed_links=len(self.fixed_links),
            loops=self.loop_count,
            components=self.component_count,
            space=space,
            plane_normal=normal,
            grubler=grubler,
            screw=screw,
            actuated=actuated,
            remaining=remaining,
            warnings=tuple(warnings),
        )

    def _loop_sequences(self) -> List[List[Tuple[Joint, int]]]:
        """Independent loops as ordered signed joint sequences."""
        if not self.links:
            return []
        uf = _UnionFind(self.links)
        tree_edges: List[Joint] = []
        loop_edges: List[Joint] = []
        for joint in self.joints:
            if uf.union(joint.port_a.link, joint.port_b.link):
                tree_edges.append(joint)
            else:
                loop_edges.append(joint)
        adjacency: Dict[str, List[Tuple[Link, Joint]]] = {
            link.name: [] for link in self.links
        }
        for joint in tree_edges:
            a, b = joint.port_a.link, joint.port_b.link
            adjacency[a.name].append((b, joint))
            adjacency[b.name].append((a, joint))

        parent: Dict[str, Tuple[Link, Joint]] = {}
        visited: set = set()
        roots: List[Link] = []
        for component in self._components():
            root = next(
                (link for link in component if link.role is LinkRole.FIXED),
                component[0],
            )
            roots.append(root)
        for root in roots:
            if root.name in visited:
                continue
            visited.add(root.name)
            queue = [root]
            while queue:
                current = queue.pop(0)
                for neighbour, joint in adjacency[current.name]:
                    if neighbour.name in visited:
                        continue
                    visited.add(neighbour.name)
                    parent[neighbour.name] = (current, joint)
                    queue.append(neighbour)

        def path_to_root(link: Link) -> List[Tuple[Joint, Link]]:
            steps: List[Tuple[Joint, Link]] = []
            current = link
            guard = 0
            while current.name in parent:
                previous, joint = parent[current.name]
                steps.append((joint, current))
                current = previous
                guard += 1
                if guard > self.link_count + 1:
                    raise RuntimeError("cycle while building kinematic tree")
            return steps

        loops: List[List[Tuple[Joint, int]]] = []
        for joint in loop_edges:
            a, b = joint.port_a.link, joint.port_b.link
            up_a = path_to_root(a)
            up_b = path_to_root(b)
            steps: List[Tuple[Joint, int]] = []
            for path_joint, from_link in up_a:
                steps.append((path_joint, self._step_sign(path_joint, from_link)))
            for path_joint, _from in reversed(up_b):
                # reverse traversal: enter the joint at the node that was
                # reached from the root side
                to_link = _to_link(path_joint, _from)
                steps.append((path_joint, self._step_sign(path_joint, to_link)))
            steps.append((joint, self._step_sign(joint, b)))
            loops.append(steps)
        return loops

    @staticmethod
    def _step_sign(joint: Joint, from_link: Link) -> int:
        return 1 if from_link is joint.port_a.link else -1

    def _residual(self, loops: Sequence[Sequence[Tuple[Joint, int]]], scale: float) -> List[float]:
        """Closure residual of every independent loop.

        Tree joints are consistent by construction, so one residual per loop
        is enough: the mismatch of the non-tree ("loop") joint.  Link offsets
        enter through the assembled poses, which makes this a true closure
        condition rather than a product of joint rotations alone.
        """
        poses = self.poses()
        result: List[float] = []
        for steps in loops:
            joint = steps[-1][0]
            frame_a = poses[joint.port_a.link.name] @ Transform.translation(
                joint.port_a.link.port(joint.port_a.port)
            )
            frame_b = poses[joint.port_b.link.name] @ Transform.translation(
                joint.port_b.link.port(joint.port_b.port)
            )
            delta = joint.motion.inverse() @ (frame_a.inverse() @ frame_b)
            omega = rotation_log(delta.R)
            result.extend(
                [
                    delta.t[0],
                    delta.t[1],
                    delta.t[2],
                    omega[0] * scale,
                    omega[1] * scale,
                    omega[2] * scale,
                ]
            )
        return result

    # ------------------------------------------------------------------
    # inverse kinematics / assembly
    # ------------------------------------------------------------------
    def solve(
        self,
        driver: Optional[Union[str, Joint]] = None,
        value: Optional[float] = None,
        *,
        drivers: Optional[Dict[Union[str, Joint], float]] = None,
        tol: Optional[float] = None,
        max_iter: int = 80,
    ) -> SolveResult:
        """Close the kinematic loops for prescribed inputs.

        A single input can be given as ``driver`` and ``value``.  For
        multi-DOF mechanisms pass a mapping, for example
        ``drivers={"J1": 30, "J2": 50}``.  The remaining joint coordinates
        are found by a damped Gauss-Newton iteration.
        """
        pinned: Dict[Joint, float] = {}
        if drivers:
            if not isinstance(drivers, dict):
                raise TypeError("drivers must be a mapping of joint -> value")
            for key, drive_value in drivers.items():
                joint = self.joint(key)  # type: ignore[arg-type]
                self._check_limit(joint, drive_value)
                joint.set_value(drive_value)
                pinned[joint] = float(drive_value)
        if driver is not None:
            driver_joint = self.joint(driver)  # type: ignore[arg-type]
            if value is not None:
                self._check_limit(driver_joint, value)
                if (
                    driver_joint in pinned
                    and abs(pinned[driver_joint] - value) > 1e-12
                ):
                    raise ValueError(
                        f"joint '{driver_joint.name}' is pinned to two different values"
                    )
                driver_joint.set_value(value)
                pinned[driver_joint] = float(value)
            elif driver_joint not in pinned:
                pinned[driver_joint] = driver_joint.value
        elif value is not None:
            raise ValueError("value requires a driver joint")
        free = [
            joint
            for joint in self.joints
            if not self._is_locked(joint) and joint not in pinned
        ]
        loops = self._loop_sequences()
        scale = self.characteristic_length
        tolerance = 1e-8 * scale if tol is None else float(tol)
        initial = [joint.value for joint in free]
        best_x = list(initial)
        best_residual = float("inf")
        best_iterations = 0
        restarts = 0
        seeds = [None, 0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0]
        for attempt, seed in enumerate(seeds):
            if attempt:
                restarts = attempt
            if seed is not None:
                for joint in free:
                    joint.set_value(seed if isinstance(joint, RevoluteJoint) else 0.0)
            x = [joint.value for joint in free]
            attempt_best_residual = float("inf")
            attempt_best_x = list(x)
            attempt_best_iteration = 0
            for iteration in range(max_iter):
                self._apply_values(free, x)
                residual = self._residual(loops, scale)
                residual_norm = max_abs(residual)
                if residual_norm < attempt_best_residual:
                    attempt_best_residual = residual_norm
                    attempt_best_x = list(x)
                    attempt_best_iteration = iteration
                if residual_norm <= tolerance:
                    break
                if not free:
                    break
                columns: List[List[float]] = []
                for index, joint in enumerate(free):
                    step = 1e-3 if isinstance(joint, RevoluteJoint) else 1e-4 * scale
                    plus = list(x)
                    minus = list(x)
                    plus[index] += step
                    minus[index] -= step
                    self._apply_values(free, plus)
                    residual_plus = self._residual(loops, scale)
                    self._apply_values(free, minus)
                    residual_minus = self._residual(loops, scale)
                    columns.append(
                        [
                            (residual_plus[k] - residual_minus[k]) / (2.0 * step)
                            for k in range(len(residual))
                        ]
                    )
                self._apply_values(free, x)
                normal = [
                    [
                        sum(columns[i][k] * columns[j][k] for k in range(len(residual)))
                        for j in range(len(free))
                    ]
                    for i in range(len(free))
                ]
                diagonal = max((normal[i][i] for i in range(len(free))), default=0.0)
                damping = max(1e-12 * diagonal, 1e-15)
                accepted = False
                for _ in range(14):
                    matrix = [
                        [
                            normal[i][j] + (damping if i == j else 0.0)
                            for j in range(len(free))
                        ]
                        for i in range(len(free))
                    ]
                    rhs = [
                        -sum(columns[i][k] * residual[k] for k in range(len(residual)))
                        for i in range(len(free))
                    ]
                    try:
                        delta = solve_linear(matrix, rhs)
                    except ZeroDivisionError:
                        damping = max(damping * 10.0, 1e-14)
                        continue
                    trial = [
                        self._clamp(joint, x[i] + delta[i])
                        for i, joint in enumerate(free)
                    ]
                    self._apply_values(free, trial)
                    trial_residual = max_abs(self._residual(loops, scale))
                    if (
                        trial_residual < residual_norm
                        or trial_residual <= tolerance
                    ):
                        x = trial
                        damping = max(damping * 0.1, 1e-15)
                        accepted = True
                        break
                    damping = max(damping * 10.0, 1e-14)
                if not accepted:
                    break
                if max_abs(delta) <= 1e-12 * (1.0 + max_abs(x)):
                    break
            if attempt_best_residual < best_residual:
                best_residual = attempt_best_residual
                best_x = attempt_best_x
                best_iterations = attempt_best_iteration
            if best_residual <= tolerance:
                break
        self._apply_values(free, best_x)
        converged = best_residual <= tolerance
        message = (
            "closure solved"
            if converged
            else "closure did not converge; try another driver value or initial state"
        )
        pinned_names = {joint.name: pinned_value for joint, pinned_value in pinned.items()}
        first_joint = next(iter(pinned), None)
        return SolveResult(
            converged=converged,
            residual=best_residual,
            iterations=best_iterations,
            restarts=restarts,
            driver=first_joint.name if first_joint is not None else None,
            driver_value=(
                pinned[first_joint] if first_joint is not None else None
            ),
            drivers=pinned_names,
            values={joint.name: joint.value for joint in self.joints},
            message=message,
        )

    @staticmethod
    def _apply_values(joints: Sequence[Joint], values: Sequence[float]) -> None:
        for joint, value in zip(joints, values):
            joint.set_value(value)

    @staticmethod
    def _clamp(joint: Joint, value: float) -> float:
        if joint.limits is None:
            return value
        return min(max(value, joint.limits[0]), joint.limits[1])

    @staticmethod
    def _check_limit(joint: Joint, value: float) -> None:
        if joint.limits is not None and not (
            joint.limits[0] <= value <= joint.limits[1]
        ):
            raise ValueError(
                f"value {value:g} is outside the limits "
                f"{joint.limits} of joint '{joint.name}'"
            )

    # ------------------------------------------------------------------
    # Grashof classification (planar four-bar)
    # ------------------------------------------------------------------
    def grashof(self) -> Optional[GrashofResult]:
        if self.link_count != 4 or self.joint_count != 4 or self.loop_count != 1:
            return None
        if len(self.fixed_links) != 1:
            return None
        if not all(isinstance(joint, RevoluteJoint) for joint in self.joints):
            return None
        if not all(len([j for j in self.joints if j.connects(link)]) == 2 for link in self.links):
            return None
        lengths: Dict[str, float] = {}
        for link in self.links:
            joint_ports = [
                joint.port_a if joint.port_a.link is link else joint.port_b
                for joint in self.joints
                if joint.connects(link)
            ]
            if len(joint_ports) != 2:
                return None
            p = link.port(joint_ports[0].port)
            q = link.port(joint_ports[1].port)
            lengths[link.name] = vdist(p, q)
        ordered = sorted(lengths.items(), key=lambda item: item[1])
        shortest, longest = ordered[0][0], ordered[-1][0]
        ground = self.fixed_links[0]
        neighbours = {
            joint.port_b.link.name
            if joint.port_a.link is ground
            else joint.port_a.link.name
            for joint in self.joints
            if joint.connects(ground)
        }
        s = lengths[shortest]
        l = lengths[longest]
        others = [value for name, value in lengths.items() if name not in (shortest, longest)]
        p_value, q_value = others[0], others[1]
        s_plus_l = s + l
        p_plus_q = p_value + q_value
        tolerance = 1e-9 * max(lengths.values())
        is_grashof = s_plus_l < p_plus_q - tolerance
        change_point = abs(s_plus_l - p_plus_q) <= tolerance
        if change_point:
            kind = "change-point"
        elif not is_grashof:
            kind = "triple-rocker"
        elif shortest == ground.name:
            kind = "double-crank (drag-link)"
        elif shortest in neighbours:
            kind = "crank-rocker"
        else:
            kind = "double-rocker"
        return GrashofResult(
            lengths=lengths,
            shortest=shortest,
            longest=longest,
            is_grashof=is_grashof or change_point,
            change_point=change_point,
            kind=kind,
        )

    # ------------------------------------------------------------------
    # validation and reporting
    # ------------------------------------------------------------------
    def validate(self) -> Diagnostics:
        errors: List[str] = []
        warnings: List[str] = []
        if not self.links:
            errors.append("mechanism has no links")
        if not self.joints:
            errors.append("mechanism has no joints")
        if not self.fixed_links:
            warnings.append("no fixed link; the mechanism is not grounded")
        if self.links and self.component_count > 1:
            warnings.append(
                f"mechanism has {self.component_count} disconnected components"
            )
        for link in self.links:
            if not any(joint.connects(link) for joint in self.joints):
                warnings.append(f"link '{link.name}' is not connected to any joint")
        port_use: Dict[Tuple[str, str], List[str]] = {}
        pair_use: Dict[frozenset, List[str]] = {}
        for joint in self.joints:
            for ref in (joint.port_a, joint.port_b):
                port_use.setdefault((ref.link.name, ref.port), []).append(joint.name)
            pair = frozenset({joint.port_a.link.name, joint.port_b.link.name})
            pair_use.setdefault(pair, []).append(joint.name)
            if not joint.in_limits:
                warnings.append(
                    f"joint '{joint.name}' value {joint.value:g} is outside its limits"
                )
        for (link_name, port), names in port_use.items():
            if len(names) > 1:
                warnings.append(
                    f"port '{port}' of link '{link_name}' is used by "
                    f"{len(names)} joints ({', '.join(names)}); rendering assumes "
                    "one joint per port"
                )
        for names in pair_use.values():
            if len(names) > 1:
                warnings.append(
                    f"links of joints {', '.join(names)} are connected more than once"
                )
        if self.fixed_links:
            errors_closure = self.closure_errors()
            for joint in self.joints:
                if self._is_locked(joint):
                    error = errors_closure[joint.name]
                    if error.magnitude > 1e-6 * self.characteristic_length:
                        warnings.append(
                            f"joint '{joint.name}' connects fixed links but does not "
                            "close; the fixed-link coordinates are inconsistent"
                        )
        dof = self.dof()
        actuated = self.input_count
        if actuated > dof:
            warnings.append(
                f"{actuated} actuated joints exceed the mobility {dof}; "
                "inputs are redundant"
            )
        if self.input_links and actuated == 0:
            warnings.append(
                "input link(s) exist but no joint is marked as actuated"
            )
        return Diagnostics(errors=tuple(errors), warnings=tuple(warnings))

    def describe(self) -> str:
        roles = {role: 0 for role in LinkRole}
        for link in self.links:
            roles[link.role] += 1
        kinds = {"revolute": 0, "prismatic": 0}
        for joint in self.joints:
            kinds[joint.kind] = kinds.get(joint.kind, 0) + 1
        report = self.mobility_report()
        diagnostics = self.validate()
        lines = [
            f"ctrl3d mechanism '{self.name}'",
            "  links       : {} (fixed={}, input={}, intermediate={}, output={})".format(
                self.link_count,
                roles[LinkRole.FIXED],
                roles[LinkRole.INPUT],
                roles[LinkRole.INTERMEDIATE],
                roles[LinkRole.OUTPUT],
            ),
            f"  joints      : {self.joint_count} (revolute={kinds['revolute']}, "
            f"prismatic={kinds['prismatic']})",
            f"  loops       : {self.loop_count}",
            "  mobility    : {} ({}, Gruebler), screw={}, inputs={}, remaining={}".format(
                report.grubler, report.space, report.screw, report.actuated, report.remaining
            ),
            f"  closure     : max {self.max_closure_error():.3e}",
        ]
        grashof = self.grashof()
        if grashof is not None:
            lines.append(f"  Grashof     : {grashof.summary}")
        if diagnostics.errors or diagnostics.warnings:
            for message in diagnostics.errors:
                lines.append(f"  error       : {message}")
            for message in diagnostics.warnings:
                lines.append(f"  warning     : {message}")
        else:
            lines.append("  diagnostics : ok")
        for message in report.warnings:
            if message not in diagnostics.warnings:
                lines.append(f"  note        : {message}")
        return "\n".join(lines)

    def __str__(self) -> str:
        return self.describe()

    # ------------------------------------------------------------------
    # rendering (imported lazily to keep the core dependency-free)
    # ------------------------------------------------------------------
    def to_scad(self, header: bool = True) -> str:
        from .render import mechanism_solid

        body = mechanism_solid(self).to_scad()
        if not header:
            return body
        lines = [
            f"// Generated by ctrl3d - mechanism '{self.name}'",
            f"// links={self.link_count} joints={self.joint_count} "
            f"dof={self.dof()}",
            "",
        ]
        return "\n".join(lines) + body

    def save_scad(self, path: str) -> str:
        from pathlib import Path

        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(self.to_scad(), encoding="utf-8")
        return str(target)

    def show(self):
        """Display the mechanism in the PythonSCAD viewport."""
        from .pyscad import show_solid
        from .render import mechanism_solid

        return show_solid(mechanism_solid(self))

    def export(self, path: str) -> str:
        """Export to .scad directly, or to STL/3MF/... through PythonSCAD."""
        from pathlib import Path

        target = Path(path)
        if target.suffix.lower() == ".scad":
            return self.save_scad(str(target))
        from .pyscad import export_solid
        from .render import mechanism_solid

        return export_solid(mechanism_solid(self), str(target))


def _to_link(joint: Joint, from_link: Link) -> Link:
    """Other endpoint of *joint* when leaving *from_link*."""
    return joint.port_b.link if from_link is joint.port_a.link else joint.port_a.link
