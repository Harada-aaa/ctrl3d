"""Minimal dependency-free 3D vector, rotation and matrix helpers.

The mechanism solver only needs small fixed-size matrices, so this module
implements them directly instead of depending on NumPy.  All rotations are
right handed and all angles in the public API are degrees.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]
Matrix4 = Tuple[Tuple[float, float, float, float], ...]

ZERO: Vec3 = (0.0, 0.0, 0.0)
EX: Vec3 = (1.0, 0.0, 0.0)
EY: Vec3 = (0.0, 1.0, 0.0)
EZ: Vec3 = (0.0, 0.0, 1.0)
IDENTITY3: Mat3 = (EX, EY, EZ)

_EPS = 1e-12


# ---------------------------------------------------------------------------
# vectors
# ---------------------------------------------------------------------------
def as_vec3(value: Sequence[float], name: str = "vector") -> Vec3:
    """Convert a 3 element sequence to a tuple of floats."""
    try:
        x, y, z = value  # type: ignore[misc]
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be a sequence of three numbers") from exc
    try:
        result = (float(x), float(y), float(z))
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must contain only numbers") from exc
    if not all(math.isfinite(c) for c in result):
        raise ValueError(f"{name} must contain finite numbers")
    return result


def vadd(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vscale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def vdot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vnorm(a: Vec3) -> float:
    return math.sqrt(vdot(a, a))


def vdist(a: Vec3, b: Vec3) -> float:
    return vnorm(vsub(a, b))


def vunit(a: Vec3, name: str = "vector") -> Vec3:
    n = vnorm(a)
    if n < _EPS:
        raise ValueError(f"{name} must not be the zero vector")
    return vscale(a, 1.0 / n)


def visclose(a: Vec3, b: Vec3, tol: float = 1e-9) -> bool:
    return vdist(a, b) <= tol


def vparallel(a: Vec3, b: Vec3, tol: float = 1e-9) -> bool:
    """True when two directions are parallel or anti-parallel."""
    return abs(vdot(vunit(a), vunit(b))) >= 1.0 - tol


def vperpendicular(a: Vec3, b: Vec3, tol: float = 1e-9) -> bool:
    return abs(vdot(vunit(a), vunit(b))) <= tol


# ---------------------------------------------------------------------------
# 3x3 matrices
# ---------------------------------------------------------------------------
def mat3_identity() -> Mat3:
    return IDENTITY3


def mat3_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(  # type: ignore[return-value]
        tuple(
            a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
            for j in range(3)
        )
        for i in range(3)
    )


def mat3_vec(a: Mat3, v: Vec3) -> Vec3:
    return (
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    )


def mat3_transpose(a: Mat3) -> Mat3:
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def mat3_rotation(axis: Sequence[float], angle_deg: float) -> Mat3:
    """Right handed rotation about *axis* using Rodrigues' formula."""
    x, y, z = vunit(as_vec3(axis, "axis"), "axis")
    angle = math.radians(float(angle_deg))
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def mat3_rotation_between(from_dir: Sequence[float], to_dir: Sequence[float]) -> Mat3:
    """Shortest rotation taking *from_dir* onto *to_dir*."""
    a = vunit(as_vec3(from_dir, "from_dir"), "from_dir")
    b = vunit(as_vec3(to_dir, "to_dir"), "to_dir")
    c = max(-1.0, min(1.0, vdot(a, b)))
    if c >= 1.0 - 1e-15:
        return IDENTITY3
    if c <= -1.0 + 1e-15:
        # 180 degrees: any axis perpendicular to *a* works.
        axis = vcross(a, EX if abs(a[0]) < 0.9 else EY)
        return mat3_rotation(vunit(axis), 180.0)
    axis = vcross(a, b)
    return mat3_rotation(vunit(axis), math.degrees(math.acos(c)))


def mat3_rotation_z_to(direction: Sequence[float]) -> Mat3:
    """Rotation that maps +Z onto *direction*."""
    return mat3_rotation_between(EZ, direction)


def rotation_angle(R: Mat3) -> float:
    """Rotation angle of a rotation matrix in radians (0..pi)."""
    c = (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


def rotation_angle_deg(R: Mat3) -> float:
    return math.degrees(rotation_angle(R))


def rotation_log(R: Mat3) -> Vec3:
    """Rotation vector (axis * angle in radians) of a rotation matrix.

    Handles the 0 and pi singularities that occur in folded and stretched
    linkage configurations.
    """
    angle = rotation_angle(R)
    if angle < 1e-12:
        return ZERO
    if math.pi - angle < 1e-7:
        # R + I = 2 a a^T near pi: the largest column gives the axis.
        columns = [
            (R[0][i] + (1.0 if i == 0 else 0.0),
             R[1][i] + (1.0 if i == 1 else 0.0),
             R[2][i] + (1.0 if i == 2 else 0.0))
            for i in range(3)
        ]
        axis = max(columns, key=vnorm)
        axis_u = vunit(axis)
        return vscale(axis_u, angle)
    axis = (
        R[2][1] - R[1][2],
        R[0][2] - R[2][0],
        R[1][0] - R[0][1],
    )
    return vscale(vunit(axis), angle)


# ---------------------------------------------------------------------------
# rigid transforms
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Transform:
    """Rigid body transform ``x -> R x + t``."""

    R: Mat3 = IDENTITY3
    t: Vec3 = ZERO

    @classmethod
    def identity(cls) -> "Transform":
        return cls()

    @classmethod
    def translation(cls, value: Sequence[float]) -> "Transform":
        return cls(IDENTITY3, as_vec3(value, "translation"))

    @classmethod
    def rotation(cls, axis: Sequence[float], angle_deg: float) -> "Transform":
        return cls(mat3_rotation(axis, angle_deg), ZERO)

    def __matmul__(self, other: "Transform") -> "Transform":
        return Transform(
            mat3_mul(self.R, other.R),
            vadd(self.t, mat3_vec(self.R, other.t)),
        )

    def __mul__(self, other: "Transform") -> "Transform":
        return self @ other

    def apply(self, point: Sequence[float]) -> Vec3:
        return vadd(self.t, mat3_vec(self.R, as_vec3(point)))

    def apply_vector(self, value: Sequence[float]) -> Vec3:
        return mat3_vec(self.R, as_vec3(value, "vector"))

    def inverse(self) -> "Transform":
        Rt = mat3_transpose(self.R)
        return Transform(Rt, vscale(mat3_vec(Rt, self.t), -1.0))

    def to_matrix4(self) -> Matrix4:
        return (
            (self.R[0][0], self.R[0][1], self.R[0][2], self.t[0]),
            (self.R[1][0], self.R[1][1], self.R[1][2], self.t[1]),
            (self.R[2][0], self.R[2][1], self.R[2][2], self.t[2]),
            (0.0, 0.0, 0.0, 1.0),
        )

    def almost_equal(self, other: "Transform", tol: float = 1e-9) -> bool:
        return (
            vdist(self.t, other.t) <= tol
            and rotation_angle(mat3_mul(mat3_transpose(self.R), other.R)) <= tol
        )


def matrix4_to_transform(matrix: Sequence[Sequence[float]]) -> Transform:
    rows = [list(map(float, row)) for row in matrix]
    if len(rows) != 4 or any(len(row) != 4 for row in rows):
        raise ValueError("matrix must be 4x4")
    return Transform(
        (
            (rows[0][0], rows[0][1], rows[0][2]),
            (rows[1][0], rows[1][1], rows[1][2]),
            (rows[2][0], rows[2][1], rows[2][2]),
        ),
        (rows[0][3], rows[1][3], rows[2][3]),
    )


# ---------------------------------------------------------------------------
# linear algebra for screw / Jacobian analysis
# ---------------------------------------------------------------------------
def rank(matrix: Sequence[Sequence[float]], tol: float = 1e-9) -> int:
    """Numerical rank of a dense matrix using Gaussian elimination."""
    rows: List[List[float]] = [list(map(float, row)) for row in matrix if len(row)]
    if not rows:
        return 0
    ncols = len(rows[0])
    if any(len(row) != ncols for row in rows):
        raise ValueError("ragged matrix")
    scale = max(abs(value) for row in rows for value in row)
    if scale == 0.0:
        return 0
    threshold = tol * scale
    pivot_row = 0
    for col in range(ncols):
        best = max(range(pivot_row, len(rows)), key=lambda i: abs(rows[i][col]))
        if abs(rows[best][col]) <= threshold:
            continue
        rows[pivot_row], rows[best] = rows[best], rows[pivot_row]
        pivot = rows[pivot_row][col]
        for i in range(pivot_row + 1, len(rows)):
            factor = rows[i][col] / pivot
            if factor == 0.0:
                continue
            for j in range(col, ncols):
                rows[i][j] -= factor * rows[pivot_row][j]
        pivot_row += 1
        if pivot_row == len(rows):
            break
    return pivot_row


def solve_linear(
    matrix: Sequence[Sequence[float]],
    rhs: Sequence[float],
    tol: float = 1e-14,
) -> List[float]:
    """Solve a square dense linear system with partial pivoting."""
    rows = [list(map(float, row)) for row in matrix]
    b = list(map(float, rhs))
    n = len(rows)
    if n == 0:
        return []
    if len(b) != n or any(len(row) != n for row in rows):
        raise ValueError("solve_linear expects a square matrix and matching rhs")
    scale = max((abs(value) for row in rows for value in row), default=0.0)
    threshold = tol * max(1.0, scale)
    for col in range(n):
        pivot = max(range(col, n), key=lambda i: abs(rows[i][col]))
        if abs(rows[pivot][col]) <= threshold:
            raise ZeroDivisionError("singular matrix")
        if pivot != col:
            rows[col], rows[pivot] = rows[pivot], rows[col]
            b[col], b[pivot] = b[pivot], b[col]
        div = rows[col][col]
        for j in range(col, n):
            rows[col][j] /= div
        b[col] /= div
        for i in range(n):
            if i == col:
                continue
            factor = rows[i][col]
            if factor == 0.0:
                continue
            for j in range(col, n):
                rows[i][j] -= factor * rows[col][j]
            b[i] -= factor * b[col]
    return b


def max_abs(values: Iterable[float]) -> float:
    return max((abs(v) for v in values), default=0.0)
