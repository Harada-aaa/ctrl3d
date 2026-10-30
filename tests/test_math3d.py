from __future__ import annotations

import math
import unittest

from ctrl3d.math3d import (
    Transform,
    mat3_mul,
    mat3_rotation,
    mat3_rotation_z_to,
    mat3_transpose,
    matrix4_to_transform,
    rank,
    rotation_angle,
    rotation_log,
    solve_linear,
    vcross,
    vdot,
    vnorm,
)


def assert_matrix_close(test, a, b, tol=1e-9):
    for i in range(3):
        for j in range(3):
            test.assertAlmostEqual(a[i][j], b[i][j], delta=tol)


class VectorTests(unittest.TestCase):
    def test_basic_vectors(self):
        self.assertAlmostEqual(vdot((1, 2, 3), (4, 5, 6)), 32.0)
        cross = vcross((1, 0, 0), (0, 1, 0))
        for got, expected in zip(cross, (0, 0, 1)):
            self.assertAlmostEqual(got, expected)
        self.assertAlmostEqual(vnorm((3, 4, 0)), 5.0)


class RotationTests(unittest.TestCase):
    def test_rotation_maps_axis(self):
        axis = (1, 2, 3)
        R = mat3_rotation(axis, 37)
        # A rotation maps its own axis onto itself.
        mapped = (
            R[0][0] * axis[0] + R[0][1] * axis[1] + R[0][2] * axis[2],
            R[1][0] * axis[0] + R[1][1] * axis[1] + R[1][2] * axis[2],
            R[2][0] * axis[0] + R[2][1] * axis[1] + R[2][2] * axis[2],
        )
        for got, want in zip(mapped, axis):
            self.assertAlmostEqual(got, want, delta=1e-9)

    def test_full_turn(self):
        assert_matrix_close(self, mat3_rotation((1, 2, 3), 360), mat3_rotation((1, 2, 3), 0))

    def test_rotation_inverse(self):
        R = mat3_rotation((0, 1, 0), 73)
        assert_matrix_close(self, mat3_mul(R, mat3_transpose(R)), mat3_rotation((1, 0, 0), 0))

    def test_z_to_direction(self):
        for direction in ((1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, 0, -1), (1, 2, 3)):
            R = mat3_rotation_z_to(direction)
            mapped = (R[0][2], R[1][2], R[2][2])
            unit = tuple(v / vnorm(direction) for v in direction)
            for got, want in zip(mapped, unit):
                self.assertAlmostEqual(got, want, delta=1e-9)

    def test_rotation_log_roundtrip(self):
        for axis, angle in (((0, 0, 1), 0.0), ((0, 0, 1), 30.0), ((1, 0, 0), 179.5), ((1, 1, 1), 179.9)):
            R = mat3_rotation(axis, angle)
            log = rotation_log(R)
            self.assertAlmostEqual(math.degrees(vnorm(log)), angle, delta=1e-4)
            if vnorm(log) > 1e-12:
                recovered = mat3_rotation(log, math.degrees(vnorm(log)))
                assert_matrix_close(self, recovered, R, tol=1e-6)

    def test_rotation_angle(self):
        self.assertAlmostEqual(rotation_angle(mat3_rotation((1, 0, 0), 90)), math.pi / 2)


class TransformTests(unittest.TestCase):
    def test_compose_and_inverse(self):
        a = Transform.rotation((0, 0, 1), 30) @ Transform.translation((1, 2, 3))
        b = Transform.rotation((1, 0, 0), 45) @ Transform.translation((-1, 0, 2))
        point = (0.3, -0.2, 0.7)
        combined = (a @ b).apply(point)
        sequential = a.apply(b.apply(point))
        for got, want in zip(combined, sequential):
            self.assertAlmostEqual(got, want, delta=1e-12)
        recovered = a.inverse().apply(a.apply(point))
        for got, want in zip(recovered, point):
            self.assertAlmostEqual(got, want, delta=1e-12)

    def test_matrix4_roundtrip(self):
        transform = Transform.rotation((0, 1, 0), 22) @ Transform.translation((4, 5, 6))
        self.assertTrue(matrix4_to_transform(transform.to_matrix4()).almost_equal(transform))


class LinearAlgebraTests(unittest.TestCase):
    def test_rank(self):
        self.assertEqual(rank([[1, 0], [0, 1]]), 2)
        self.assertEqual(rank([[1, 2], [2, 4]]), 1)
        self.assertEqual(rank([[0, 0], [0, 0]]), 0)
        self.assertEqual(rank([]), 0)

    def test_solve_linear(self):
        solution = solve_linear([[2, 1], [1, 3]], [5, 10])
        self.assertAlmostEqual(solution[0], 1.0)
        self.assertAlmostEqual(solution[1], 3.0)

    def test_singular_matrix(self):
        with self.assertRaises(ZeroDivisionError):
            solve_linear([[1, 2], [2, 4]], [1, 2])


if __name__ == "__main__":
    unittest.main()
