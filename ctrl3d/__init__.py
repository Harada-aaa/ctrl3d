"""ctrl3d - mechanism design toolkit for PythonSCAD.

Describe a mechanism with input, intermediate, fixed and output links,
connect them with revolute (R) or prismatic (P) pairs, then inspect its
mobility, solve its closed configuration and render it in PythonSCAD.

Quick start::

    from ctrl3d import Mechanism

    m = Mechanism("four_bar")
    ground = m.fixed("ground", spacing=0.40)
    crank = m.input("crank", length=0.12)
    coupler = m.intermediate("coupler", length=0.40)
    rocker = m.output("rocker", length=0.30)

    m.revolute(ground["A"], crank["A"])
    m.revolute(crank["B"], coupler["A"])
    m.revolute(coupler["B"], rocker["A"])
    m.revolute(rocker["B"], ground["B"])

    print(m.describe())
    print("DOF:", m.dof())
"""

from .analysis import (
    ClosureError,
    Diagnostics,
    GrashofResult,
    MobilityReport,
    SolveResult,
)
from .csg import (
    Background,
    Colored,
    Cube,
    Cylinder,
    Difference,
    Empty,
    Intersection,
    Solid,
    Sphere,
    Transformed,
    Union,
    parse_color,
)
from .joints import Joint, PrismaticJoint, RevoluteJoint
from .links import (
    FixedLink,
    InputLink,
    IntermediateLink,
    Link,
    LinkRole,
    OutputLink,
    PortRef,
    ports_along_line,
)
from .math3d import Transform, Vec3
from .mechanism import Mechanism
from .pyscad import is_available as pythonscad_available
from .render import link_solid, mechanism_solid

__version__ = "0.1.0"

__all__ = [
    "Mechanism",
    "Link",
    "FixedLink",
    "InputLink",
    "IntermediateLink",
    "OutputLink",
    "LinkRole",
    "PortRef",
    "ports_along_line",
    "Joint",
    "RevoluteJoint",
    "PrismaticJoint",
    "MobilityReport",
    "GrashofResult",
    "ClosureError",
    "Diagnostics",
    "SolveResult",
    "Transform",
    "Vec3",
    "Solid",
    "Empty",
    "Cube",
    "Sphere",
    "Cylinder",
    "Union",
    "Difference",
    "Intersection",
    "Transformed",
    "Colored",
    "Background",
    "parse_color",
    "mechanism_solid",
    "link_solid",
    "pythonscad_available",
    "__version__",
]
