from __future__ import annotations

import unittest

from ctrl3d import Mechanism
from tests.mechanisms import (
    five_bar,
    four_bar,
    slider_crank,
    spatial_arm,
    spatial_four_bar,
)


class PlanarDofTests(unittest.TestCase):
    def test_four_bar(self):
        m = four_bar()
        self.assertEqual(m.link_count, 4)
        self.assertEqual(m.joint_count, 4)
        self.assertEqual(m.loop_count, 1)
        self.assertEqual(m.dof(), 1)
        self.assertEqual(m.dof("planar"), 1)
        self.assertEqual(m.dof("spatial"), -2)  # spatial Gruebler count
        self.assertEqual(m.input_count, 1)

    def test_four_bar_screw_after_assembly(self):
        m = four_bar()
        result = m.solve("J1", 45)
        self.assertTrue(result.converged)
        self.assertEqual(m.dof("screw"), 1)
        report = m.mobility_report()
        self.assertEqual(report.grubler, 1)
        self.assertEqual(report.screw, 1)
        self.assertTrue(report.consistent)
        self.assertEqual(report.remaining, 0)
        self.assertEqual(report.space, "planar")

    def test_slider_crank(self):
        m = slider_crank()
        self.assertEqual(m.dof(), 1)
        self.assertEqual(m.dof("screw"), 1)
        self.assertEqual(m.input_count, 1)
        report = m.mobility_report()
        self.assertTrue(report.consistent)
        self.assertEqual(report.remaining, 0)

    def test_five_bar(self):
        m = five_bar()
        self.assertEqual(m.dof(), 2)
        self.assertEqual(m.input_count, 2)
        report = m.mobility_report()
        self.assertEqual(report.grubler, 2)
        self.assertEqual(report.remaining, 0)

    def test_planar_open_chain(self):
        m = Mechanism("planar_arm")
        ground = m.fixed("ground", ports={"A": (0, 0, 0)})
        first = m.input("first", length=1.0)
        second = m.intermediate("second", length=1.0)
        third = m.output("third", length=1.0)
        m.revolute(ground["A"], first["A"], name="J1")
        m.revolute(first["B"], second["A"], name="J2")
        m.revolute(second["B"], third["A"], name="J3")
        self.assertEqual(m.dof(), 3)
        self.assertEqual(m.dof("screw"), 3)
        planar, normal = m.planarity()
        self.assertTrue(planar)
        self.assertEqual(normal, (0.0, 0.0, 1.0))

    def test_ungrounded_four_bar_includes_global_dof(self):
        m = four_bar(grounded=False)
        self.assertEqual(m.dof(), 4)
        self.assertEqual(len(m.fixed_links), 0)
        self.assertTrue(any("no fixed link" in warning for warning in m.validate().warnings))


class SpatialDofTests(unittest.TestCase):
    def test_spatial_arm(self):
        m = spatial_arm()
        planar, _ = m.planarity()
        self.assertFalse(planar)
        self.assertEqual(m.dof(), 3)
        self.assertEqual(m.dof("screw"), 3)
        self.assertEqual(m.input_count, 1)

    def test_spatial_four_bar_overconstrained(self):
        m = spatial_four_bar()
        self.assertFalse(m.planarity()[0])
        self.assertEqual(m.dof(), -2)
        report = m.mobility_report()
        self.assertEqual(report.space, "spatial")
        self.assertEqual(report.grubler, -2)
        self.assertTrue(any("negative" in warning for warning in report.warnings))

    def test_invalid_space(self):
        with self.assertRaises(ValueError):
            four_bar().dof("hyperbolic")


class GrashofTests(unittest.TestCase):
    def test_crank_rocker(self):
        result = four_bar().grashof()
        self.assertIsNotNone(result)
        self.assertTrue(result.is_grashof)
        self.assertFalse(result.change_point)
        self.assertEqual(result.shortest, "crank")
        self.assertEqual(result.kind, "crank-rocker")

    def test_double_crank(self):
        result = four_bar(
            ground_length=0.10,
            crank_length=0.40,
            coupler_length=0.30,
            rocker_length=0.35,
        ).grashof()
        self.assertEqual(result.kind, "double-crank (drag-link)")

    def test_double_rocker(self):
        result = four_bar(
            ground_length=0.40,
            crank_length=0.30,
            coupler_length=0.10,
            rocker_length=0.35,
        ).grashof()
        self.assertEqual(result.kind, "double-rocker")

    def test_non_grashof(self):
        result = four_bar(
            ground_length=0.40,
            crank_length=0.30,
            coupler_length=0.20,
            rocker_length=0.15,
        ).grashof()
        self.assertFalse(result.is_grashof)
        self.assertEqual(result.kind, "triple-rocker")

    def test_change_point(self):
        result = four_bar(
            ground_length=0.40,
            crank_length=0.20,
            coupler_length=0.30,
            rocker_length=0.30,
        ).grashof()
        self.assertTrue(result.change_point)
        self.assertEqual(result.kind, "change-point")

    def test_slider_crank_has_no_grashof_classification(self):
        self.assertIsNone(slider_crank().grashof())


if __name__ == "__main__":
    unittest.main()
