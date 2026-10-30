"""Five-bar linkage with two inputs.

Demonstrates a multi-DOF mechanism: the Gruebler mobility is 2, so two joint
coordinates must be prescribed to close the loop.  ``solve`` accepts a
mapping of driven joints for this purpose.
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
    m = Mechanism("five_bar")

    ground = m.fixed("ground", ports={"A": (0.0, 0.0, 0.0), "B": (0.50, 0.0, 0.0)})
    left = m.input("crank_left", length=0.16)
    right = m.input("crank_right", length=0.18)
    coupler = m.intermediate("coupler", length=0.40)
    platform = m.output("platform", ports={"A": (0.0, 0.0, 0.0), "B": (0.25, 0.0, 0.0)})

    m.revolute(ground["A"], left["A"], name="J1")
    m.revolute(ground["B"], right["A"], name="J2")
    m.revolute(left["B"], coupler["A"], name="J3")
    m.revolute(coupler["B"], platform["A"], name="J4")
    m.revolute(right["B"], platform["B"], name="J5")
    return m


def main() -> None:
    m = build()

    result = m.solve(drivers={"J1": 30.0, "J2": 50.0})
    print(result)
    print(m.describe())
    print()
    print(f"closure error: {m.max_closure_error():.3e}")

    if is_available():
        m.show()
    else:
        target = HERE / "five_bar.scad"
        m.save_scad(str(target))
        print(f"PythonSCAD is not running; wrote {target}")


if __name__ == "__main__":
    main()
