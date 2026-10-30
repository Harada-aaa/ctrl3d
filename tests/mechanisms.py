from __future__ import annotations

from ctrl3d import Mechanism


def four_bar(
    ground_length=0.40,
    crank_length=0.12,
    coupler_length=0.40,
    rocker_length=0.30,
    grounded=True,
) -> Mechanism:
    m = Mechanism("four_bar")
    make_ground = m.fixed if grounded else m.intermediate
    ground = make_ground("ground", spacing=ground_length)
    crank = m.input("crank", length=crank_length)
    coupler = m.intermediate("coupler", length=coupler_length)
    rocker = m.output("rocker", length=rocker_length)
    m.revolute(ground["A"], crank["A"], name="J1")
    m.revolute(crank["B"], coupler["A"], name="J2")
    m.revolute(coupler["B"], rocker["A"], name="J3")
    m.revolute(rocker["B"], ground["B"], name="J4")
    return m


def slider_crank() -> Mechanism:
    m = Mechanism("slider_crank")
    ground = m.fixed("ground", ports={"A": (0.0, 0.0, 0.0), "B": (0.40, 0.0, 0.0)})
    crank = m.input("crank", length=0.10)
    rod = m.intermediate("rod", length=0.30)
    slider = m.output("slider", ports={"A": (0.0, 0.0, 0.0), "B": (0.0, 0.0, 0.0)})
    m.revolute(ground["A"], crank["A"], name="J1")
    m.revolute(crank["B"], rod["A"], name="J2")
    m.revolute(rod["B"], slider["A"], name="J3")
    m.prismatic(ground["B"], slider["B"], axis=(1, 0, 0), limits=(-0.25, 0.10), name="J4")
    return m


def five_bar() -> Mechanism:
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


def spatial_arm() -> Mechanism:
    m = Mechanism("spatial_arm")
    ground = m.fixed("ground", ports={"A": (0.0, 0.0, 0.0)})
    first = m.input("link1", length=0.20)
    second = m.intermediate("link2", length=0.25)
    third = m.output("link3", length=0.15)
    m.revolute(ground["A"], first["A"], axis=(0, 0, 1), name="J1")
    m.revolute(first["B"], second["A"], axis=(1, 0, 0), name="J2")
    m.revolute(second["B"], third["A"], axis=(0, 1, 0), name="J3")
    return m


def spatial_four_bar() -> Mechanism:
    m = Mechanism("spatial_four_bar")
    ground = m.fixed("ground", spacing=0.40)
    first = m.input("first", length=0.15)
    second = m.intermediate("second", length=0.40)
    third = m.output("third", length=0.30)
    m.revolute(ground["A"], first["A"], axis=(0, 0, 1), name="J1")
    m.revolute(first["B"], second["A"], axis=(1, 0, 0), name="J2")
    m.revolute(second["B"], third["A"], axis=(0, 0, 1), name="J3")
    m.revolute(third["B"], ground["B"], axis=(0, 0, 1), name="J4")
    return m
