from __future__ import annotations

import unittest

from ctrl3d import Mechanism
from tests.mechanisms import four_bar


class ValidationTests(unittest.TestCase):
    def test_valid_four_bar(self):
        diagnostics = four_bar().validate()
        self.assertTrue(diagnostics.ok)
        self.assertEqual(diagnostics.warnings, ())
        self.assertTrue(diagnostics)
        self.assertEqual(diagnostics.format(), "ok")

    def test_no_fixed_link(self):
        m = four_bar(grounded=False)
        diagnostics = m.validate()
        self.assertTrue(any("no fixed link" in warning for warning in diagnostics.warnings))

    def test_empty_mechanism(self):
        diagnostics = Mechanism("empty").validate()
        self.assertFalse(diagnostics.ok)
        self.assertTrue(any("no links" in error for error in diagnostics.errors))
        self.assertTrue(any("no joints" in error for error in diagnostics.errors))

    def test_disconnected_link(self):
        m = four_bar()
        m.intermediate("lonely", length=0.1)
        diagnostics = m.validate()
        self.assertTrue(any("disconnected" in warning for warning in diagnostics.warnings))
        self.assertTrue(any("not connected" in warning for warning in diagnostics.warnings))

    def test_reused_port_warning(self):
        m = four_bar()
        extra = m.intermediate("extra", length=0.1)
        m.revolute("ground.A", extra["A"], axis=(0, 0, 1))
        diagnostics = m.validate()
        self.assertTrue(any("used by 2 joints" in warning for warning in diagnostics.warnings))

    def test_pair_reuse_warning(self):
        m = Mechanism("pair_reuse")
        ground = m.fixed(
            "ground", ports={"A": (0, 0, 0), "B": (1, 0, 0), "C": (0, 1, 0)}
        )
        crank = m.input("crank", length=0.4)
        m.revolute(ground["A"], crank["A"], name="J1")
        m.revolute(ground["C"], crank["B"], name="J2")
        diagnostics = m.validate()
        self.assertTrue(
            any("connected more than once" in warning for warning in diagnostics.warnings)
        )

    def test_duplicate_joint_rejected(self):
        m = four_bar()
        with self.assertRaises(ValueError):
            m.revolute(m.link("ground")["A"], m.link("crank")["A"])

    def test_out_of_limits_warning(self):
        m = four_bar()
        m.joint("J1").limits = (-30, 30)
        m.joint("J1").value = 60
        diagnostics = m.validate()
        self.assertTrue(any("outside its limits" in warning for warning in diagnostics.warnings))

    def test_over_actuation_warning(self):
        m = four_bar()
        m.joint("J2").actuated = True
        diagnostics = m.validate()
        self.assertTrue(any("exceed the mobility" in warning for warning in diagnostics.warnings))

    def test_input_without_actuator_warning(self):
        m = four_bar()
        m.joint("J1").actuated = False
        diagnostics = m.validate()
        self.assertTrue(
            any("no joint is marked as actuated" in warning for warning in diagnostics.warnings)
        )


class ConstructionTests(unittest.TestCase):
    def test_endpoint_forms(self):
        m = Mechanism("forms")
        ground = m.fixed("ground", spacing=1.0)
        crank = m.input("crank", length=0.4)
        m.revolute("ground.A", "crank.A", name="J1")
        m.revolute((ground, "B"), (crank, "B"), name="J2")
        self.assertEqual(m.joint_count, 2)
        self.assertEqual(str(m.joint("J1").port_a), "ground.A")

    def test_single_port_link_endpoint(self):
        m = Mechanism("single")
        ground = m.fixed("ground", ports={"A": (0, 0, 0)})
        output = m.output("slider", ports={"A": (0, 0, 0)})
        m.prismatic(ground["A"], output, axis=(1, 0, 0))
        self.assertEqual(m.joint_count, 1)

    def test_ambiguous_link_endpoint(self):
        m = Mechanism("ambiguous")
        ground = m.fixed("ground", spacing=1.0)
        crank = m.input("crank", length=0.4)
        with self.assertRaises(ValueError):
            m.revolute(ground, crank["A"])

    def test_unregistered_link_rejected(self):
        m = Mechanism("registered")
        ground = m.fixed("ground", spacing=1.0)
        foreign = m.input("foreign", length=0.4)
        m2 = Mechanism("other")
        with self.assertRaises(ValueError):
            m2.revolute(ground["A"], foreign["A"])

    def test_auto_actuation(self):
        m = four_bar()
        self.assertTrue(m.joint("J1").actuated)
        self.assertFalse(m.joint("J4").actuated)
        self.assertEqual(m.input_count, 1)

    def test_describe_contains_design_summary(self):
        text = four_bar().describe()
        for token in ("links", "joints", "mobility", "Grashof"):
            self.assertIn(token, text)
        self.assertIn("crank-rocker", text)


if __name__ == "__main__":
    unittest.main()
