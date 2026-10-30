from __future__ import annotations

import os
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from ctrl3d import parse_color
from ctrl3d.csg import Cube, Empty, Union
from tests.mechanisms import four_bar, slider_crank


class FakeSolid:
    def __init__(self, kind):
        self.kind = kind

    def __repr__(self):
        return f"FakeSolid({self.kind})"


def fake_pythonscad():
    module = types.ModuleType("pythonscad")
    calls = []

    def make(name):
        def function(*args, **kwargs):
            calls.append((name, args, kwargs))
            return FakeSolid(name)

        return function

    for name in (
        "cube",
        "sphere",
        "cylinder",
        "union",
        "difference",
        "intersection",
        "color",
        "multmatrix",
        "background",
        "show",
        "export",
    ):
        setattr(module, name, make(name))
    module.calls = calls
    return module


class ScadOutputTests(unittest.TestCase):
    def test_four_bar_scad(self):
        m = four_bar()
        m.solve("J1", 45)
        text = m.to_scad()
        self.assertIn("multmatrix(", text)
        self.assertIn("cylinder(", text)
        self.assertIn("sphere(", text)
        self.assertIn("color(", text)
        self.assertIn("mechanism 'four_bar'", text)
        self.assertEqual(text.count("{"), text.count("}"))
        self.assertTrue(text.rstrip().endswith("}"))

    def test_prismatic_geometry_contains_cube(self):
        text = slider_crank().to_scad()
        self.assertIn("cube(", text)
        self.assertIn("cylinder(", text)

    def test_save_scad(self):
        m = four_bar()
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "sub", "four_bar.scad")
            result = m.save_scad(path)
            self.assertEqual(result, path)
            with open(path, encoding="utf-8") as handle:
                text = handle.read()
            self.assertIn("union()", text)

    def test_header_can_be_disabled(self):
        self.assertFalse(four_bar().to_scad(header=False).startswith("//"))


class CsgTests(unittest.TestCase):
    def test_empty(self):
        empty = Empty()
        self.assertTrue(empty.is_empty)
        self.assertIn("empty", empty.to_scad())
        self.assertTrue(Union([empty]).is_empty)

    def test_parse_color_hex(self):
        rgb, alpha = parse_color("#ff8000")
        self.assertAlmostEqual(rgb[0], 1.0)
        self.assertAlmostEqual(rgb[1], 128 / 255)
        self.assertAlmostEqual(rgb[2], 0.0)
        self.assertEqual(alpha, 1.0)
        rgb, alpha = parse_color("#ff800080")
        self.assertAlmostEqual(alpha, 128 / 255)
        with self.assertRaises(ValueError):
            parse_color("#fff")

    def test_cube_scad(self):
        self.assertIn("cube(size=[1, 2, 3]", Cube((1, 2, 3)).to_scad())


class PythonScadBridgeTests(unittest.TestCase):
    def test_show_calls_pythonscad(self):
        fake = fake_pythonscad()
        with patch.dict(sys.modules, {"pythonscad": fake}):
            four_bar().show()
        names = [call[0] for call in fake.calls]
        self.assertIn("show", names)
        self.assertIn("cylinder", names)
        self.assertIn("sphere", names)
        self.assertIn("multmatrix", names)
        self.assertIn("color", names)
        color_calls = [call for call in fake.calls if call[0] == "color"]
        self.assertTrue(color_calls)
        for _, args, _ in color_calls:
            self.assertEqual(len(args), 2)
            self.assertEqual(len(args[1]), 4)  # RGBA list, not alpha= keyword

    def test_show_prismatic_uses_cube(self):
        fake = fake_pythonscad()
        with patch.dict(sys.modules, {"pythonscad": fake}):
            slider_crank().show()
        names = [call[0] for call in fake.calls]
        self.assertIn("cube", names)

    def test_export_through_pythonscad(self):
        fake = fake_pythonscad()
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "sub", "four_bar.stl")
            with patch.dict(sys.modules, {"pythonscad": fake}):
                result = four_bar().export(path)
            self.assertEqual(result, path)
            self.assertTrue(os.path.isdir(os.path.dirname(path)))
        names = [call[0] for call in fake.calls]
        self.assertIn("export", names)

    def test_scad_export_does_not_need_pythonscad(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "four_bar.scad")
            with patch.dict(sys.modules, {"pythonscad": None}):
                result = four_bar().export(path)
            self.assertEqual(result, path)
            self.assertTrue(os.path.exists(path))

    def test_show_without_pythonscad_raises(self):
        with patch.dict(sys.modules, {"pythonscad": None}):
            with self.assertRaises(RuntimeError):
                four_bar().show()


if __name__ == "__main__":
    unittest.main()
