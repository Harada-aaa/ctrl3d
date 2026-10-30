from __future__ import annotations

import unittest

from ctrl3d import (
    FixedLink,
    InputLink,
    IntermediateLink,
    Link,
    LinkRole,
    OutputLink,
    PortRef,
    PrismaticJoint,
    RevoluteJoint,
    ports_along_line,
)


class LinkTests(unittest.TestCase):
    def test_length_ports(self):
        link = Link("bar", length=2.0)
        self.assertEqual(link.port("A"), (0.0, 0.0, 0.0))
        self.assertEqual(link.port("B"), (2.0, 0.0, 0.0))
        self.assertAlmostEqual(link.length, 2.0)
        self.assertAlmostEqual(link.extent, 2.0)

    def test_explicit_ports(self):
        link = Link("plate", ports={"A": (0, 0, 0), "B": (1, 0, 0), "C": (0, 2, 0)})
        self.assertAlmostEqual(link.extent, (1 + 4) ** 0.5)

    def test_port_ref(self):
        link = Link("bar", length=1.0)
        ref = link["A"]
        self.assertIsInstance(ref, PortRef)
        self.assertIs(ref.link, link)
        self.assertEqual(ref.port, "A")
        self.assertEqual(str(ref), "bar.A")
        with self.assertRaises(KeyError):
            link["missing"]

    def test_roles(self):
        self.assertIs(Link("a", length=1).role, LinkRole.INTERMEDIATE)
        self.assertIs(FixedLink("g", {"A": (0, 0, 0), "B": (2, 0, 0)}).role, LinkRole.FIXED)
        self.assertIs(InputLink("i", length=1).role, LinkRole.INPUT)
        self.assertIs(IntermediateLink("c", length=1).role, LinkRole.INTERMEDIATE)
        self.assertIs(OutputLink("o", length=1).role, LinkRole.OUTPUT)

    def test_length_validation(self):
        with self.assertRaises(ValueError):
            Link("bad", length=0)
        with self.assertRaises(ValueError):
            Link("bad", ports={"A": (0, 0, 0)}, length=1)
        with self.assertRaises(ValueError):
            Link("bad")
        with self.assertRaises(TypeError):
            Link("bad", ports={"A": (0, 0)})

    def test_ports_along_line(self):
        ports = ports_along_line(2.0, 3)
        self.assertEqual(tuple(ports), ("A", "B", "C"))
        self.assertEqual(ports["C"], (2.0, 0.0, 0.0))


class JointTests(unittest.TestCase):
    def setUp(self):
        self.a = Link("a", length=1.0)
        self.b = Link("b", length=1.0)

    def test_revolute_motion(self):
        joint = RevoluteJoint("r", self.a["B"], self.b["A"], axis=(0, 0, 2), value=90)
        self.assertEqual(joint.axis, (0.0, 0.0, 1.0))
        self.assertEqual(joint.angle, 90.0)
        self.assertAlmostEqual(joint.motion.apply((1, 0, 0))[1], 1.0)
        self.assertEqual(joint.dof, 1)
        self.assertEqual(joint.kind, "revolute")

    def test_prismatic_motion(self):
        joint = PrismaticJoint("p", self.a["B"], self.b["A"], axis=(2, 0, 0), value=0.5)
        self.assertEqual(joint.displacement, 0.5)
        self.assertEqual(joint.motion.apply((0, 0, 0)), (0.5, 0.0, 0.0))
        self.assertEqual(joint.kind, "prismatic")

    def test_limits(self):
        joint = RevoluteJoint("r", self.a["A"], self.b["A"], limits=(-30, 30), value=10)
        self.assertTrue(joint.in_limits)
        joint.value = 40
        self.assertFalse(joint.in_limits)
        joint.clamp()
        self.assertEqual(joint.value, 30)
        with self.assertRaises(ValueError):
            RevoluteJoint("bad", self.a["A"], self.b["A"], limits=(10, -10))

    def test_same_link_rejected(self):
        with self.assertRaises(ValueError):
            RevoluteJoint("bad", self.a["A"], self.a["B"])

    def test_unknown_port_rejected(self):
        with self.assertRaises(KeyError):
            RevoluteJoint("bad", self.a["Z"], self.b["A"])

    def test_connecting_links(self):
        joint = RevoluteJoint("r", self.a["A"], self.b["A"])
        self.assertTrue(joint.connects(self.a))
        self.assertIs(joint.other_link(self.a), self.b)
        a, b = joint.links
        self.assertIs(a, self.a)
        self.assertIs(b, self.b)


if __name__ == "__main__":
    unittest.main()
