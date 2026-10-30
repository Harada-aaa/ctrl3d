"""Spatial 3R arm: an open chain with three revolute pairs.

Shows the difference between planar and spatial mobility and produces a
three-dimensional PythonSCAD model.
"""

from pathlib import Path
import sys

try:
    HERE = Path(__file__).resolve().parent
except NameError:  # PythonSCAD executes scripts without __file__
    HERE = Path.cwd()
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))

from ctrl3d import Mechanism
from ctrl3d.pyscad import is_available


def build() -> Mechanism:
    m = Mechanism("spatial_arm")

    base = m.fixed("base", ports={"A": (0.0, 0.0, 0.0)})
    upper = m.input("upper_arm", length=0.22)
    fore = m.intermediate("fore_arm", length=0.18)
    hand = m.output("hand", length=0.10)

    m.revolute(base["A"], upper["A"], axis=(0.0, 0.0, 1.0), value=25.0, name="J1")
    m.revolute(upper["B"], fore["A"], axis=(1.0, 0.0, 0.0), value=-35.0, name="J2")
    m.revolute(fore["B"], hand["A"], axis=(0.0, 1.0, 0.0), value=40.0, name="J3")
    return m


def main() -> None:
    m = build()
    print(m.describe())
    print(f"tool point: {m.point('hand', 'B')}")

    if is_available():
        m.show()
    else:
        target = HERE / "spatial_arm.scad"
        m.save_scad(str(target))
        print(f"PythonSCAD is not running; wrote {target}")


if __name__ == "__main__":
    main()
