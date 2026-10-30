from __future__ import annotations

import math
import unittest

from ctrl3d import Mechanism
from tests.mechanisms import four_bar, slider_crank


class FourBarKinematicsTests(unittest.TestCase):
    def test_solve_closes_loop_for_several_angles(self):
        m = four_bar()
        for angle in (0.0, 30.0, 60.0, 120.0, 240.0, 315.0):
            result = m.solve("J1", angle)
            self.assertTrue(result.converged, f"angle {angle}: {result}")
            self.assertLess(m.max_closure_error(), 1e-8)
            # The coupler keeps its length and the rocker stays pinned to ground.
            coupler_a = m.point("coupler", "A")
            coupler_b = m.point("coupler", "B")
            self.assertAlmostEqual(math.dist(coupler_a, coupler_b), 0.40, delta=1e-8)
            rocker_b = m.point("rocker", "B")
            self.assertAlmostEqual(math.dist(rocker_b, (0.40, 0.0, 0.0)), 1e-8)
            self.assertAlmostEqual(m.point("crank", "B")[0] ** 2 + m.point("crank", "B")[1] ** 2, 0.12 ** 2, delta=1e-8)

    def test_driver_limit(self):
        m = four_bar()
        m.joint("J1").limits = (-90.0, 90.0)
        with self.assertRaises(ValueError):
            m.solve("J1", 120.0)

    def test_closed_state_is_reusable(self):
        m = four_bar()
        m.solve("J1", 60)
        first = m.state()
        self.assertTrue(m.solve("J1", 60).converged)
        for name, value in first.items():
            self.assertAlmostEqual(m.joint(name).value, value, delta=1e-6)


class SliderCrankKinematicsTests(unittest.TestCase):
    def test_displacement(self):
        m = slider_crank()
        crank = 0.10
        rod = 0.30
        for angle in (0.0, 45.0, 90.0, 135.0):
            result = m.solve("J1", angle)
            self.assertTrue(result.converged, f"angle {angle}: {result}")
            theta = math.radians(angle)
            expected_x = crank * math.cos(theta) + math.sqrt(
                rod ** 2 - (crank * math.sin(theta)) ** 2
            )
            expected_displacement = expected_x - 0.40
            self.assertAlmostEqual(
                m.joint("J4").value, expected_displacement, delta=1e-7
            )
            self.assertLess(m.max_closure_error(), 1e-8)

    def test_slider_position(self):
        m = slider_crank()
        m.solve("J1", 90)
        point = m.point("slider", "A")
        self.assertAlmostEqual(point[0], math.sqrt(0.30 ** 2 - 0.10 ** 2), delta=1e-8)
        self.assertAlmostEqual(point[1], 0.0, delta=1e-8)


class FiveBarKinematicsTests(unittest.TestCase):
    def test_two_inputs_close_the_loop(self):
        from tests.mechanisms import five_bar

        m = five_bar()
        result = m.solve(drivers={"J1": 30.0, "J2": 50.0})
        self.assertTrue(result.converged, str(result))
        self.assertAlmostEqual(m.joint("J1").value, 30.0)
        self.assertAlmostEqual(m.joint("J2").value, 50.0)
        self.assertLess(m.max_closure_error(), 1e-8)
        self.assertEqual(m.screw_dof(), 2)


class GenericKinematicsTests(unittest.TestCase):
    def test_open_chain_has_no_closure_constraint(self):
        m = Mechanism("arm")
        ground = m.fixed("ground", ports={"A": (0, 0, 0)})
        first = m.input("first", length=0.3)
        second = m.output("second", length=0.2)
        m.revolute(ground["A"], first["A"], name="J1")
        m.revolute(first["B"], second["A"], name="J2", value=30)
        result = m.solve("J1", 20)
        self.assertTrue(result.converged)
        self.assertEqual(m.joint("J2").value, 30.0)
        self.assertAlmostEqual(m.joint("J1").value, 20.0)

    def test_state_helpers_and_pose(self):
        m = four_bar()
        m.set_state({"J1": 10})
        self.assertAlmostEqual(m.joint("J1").value, 10.0)
        pose = m.poses()["crank"]
        point = pose.apply((0.12, 0, 0))
        self.assertAlmostEqual(point[0], 0.12 * math.cos(math.radians(10)), delta=1e-12)
        self.assertAlmostEqual(point[1], 0.12 * math.sin(math.radians(10)), delta=1e-12)


if __name__ == "__main__":
    unittest.main()
