"""Four-bar linkage: input, coupler, output and fixed link.

Run this file inside PythonSCAD to see the mechanism, or run it with plain
Python to print the design analysis and write ``examples/four_bar.scad``.
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
    m = Mechanism("four_bar")

    # 固定節 (ground), 入力節 (crank), 中間節 (coupler), 出力節 (rocker)
    ground = m.fixed("ground", spacing=0.40)
    crank = m.input("crank", length=0.12)
    coupler = m.intermediate("coupler", length=0.40)
    rocker = m.output("rocker", length=0.30)

    # 回転対偶 (R pairs): ground-crank-coupler-rocker-ground
    m.revolute(ground["A"], crank["A"], name="J1")
    m.revolute(crank["B"], coupler["A"], name="J2")
    m.revolute(coupler["B"], rocker["A"], name="J3")
    m.revolute(rocker["B"], ground["B"], name="J4")
    return m


def main() -> None:
    m = build()

    result = m.solve("J1", 45.0)
    print(result)
    print(m.describe())
    print()
    print("Grashof:", m.grashof())
    print(f"closure error: {m.max_closure_error():.3e}")
    print(f"output pin B : {m.point('rocker', 'B')}")

    if is_available():
        m.show()
    else:
        target = HERE / "four_bar.scad"
        m.save_scad(str(target))
        print(f"PythonSCAD is not running; wrote {target}")


if __name__ == "__main__":
    main()
