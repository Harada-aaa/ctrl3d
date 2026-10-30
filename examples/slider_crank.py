"""Slider-crank mechanism: revolute input plus one prismatic pair.

The output link is a slider guided by a 直進対偶 (prismatic pair).
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
    m = Mechanism("slider_crank")

    ground = m.fixed("ground", ports={"A": (0.0, 0.0, 0.0), "B": (0.40, 0.0, 0.0)})
    crank = m.input("crank", length=0.10)
    rod = m.intermediate("rod", length=0.30)
    slider = m.output("slider", ports={"A": (0.0, 0.0, 0.0), "B": (0.0, 0.0, 0.0)})

    m.revolute(ground["A"], crank["A"], name="J1")
    m.revolute(crank["B"], rod["A"], name="J2")
    m.revolute(rod["B"], slider["A"], name="J3")
    m.prismatic(
        ground["B"],
        slider["B"],
        axis=(1.0, 0.0, 0.0),
        limits=(-0.25, 0.10),
        name="J4",
    )
    return m


def main() -> None:
    m = build()
    print(m.describe())

    result = m.solve("J1", 90.0)
    print(result)
    print(f"slider displacement: {m.joint('J4').value:.6f}")
    print(f"slider position    : {m.point('slider', 'A')}")

    if is_available():
        m.show()
    else:
        target = HERE / "slider_crank.scad"
        m.save_scad(str(target))
        print(f"PythonSCAD is not running; wrote {target}")


if __name__ == "__main__":
    main()
