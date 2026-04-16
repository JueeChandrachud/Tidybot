#!/usr/bin/env python3
"""
compute_inertia.py

Computes mass and diagonal inertia tensor (Ixx, Iyy, Izz) for primitive
shapes from geometry + density, and emits a URDF-ready <inertial> block.

Formulas (solid, uniform-density primitives, principal axes at centroid):

  Box (l along x, w along y, h along z):
    m    = rho * l * w * h
    Ixx  = m/12 * (w^2 + h^2)
    Iyy  = m/12 * (l^2 + h^2)
    Izz  = m/12 * (l^2 + w^2)

  Cylinder (radius r, length h, symmetry axis = Z by default):
    m    = rho * pi * r^2 * h
    Izz  = 1/2 * m * r^2                (about symmetry axis)
    Ixx  = Iyy = m/12 * (3r^2 + h^2)    (transverse axes)

For wheels in this project the symmetry axis is Y (rotation axis), so the
--axis flag swaps the tensor accordingly.

Used to populate every <inertial> block in the tidybot URDF so that no
value is guessed. Run examples:

  python3 compute_inertia.py box --l 0.45 --w 0.35 --h 0.12 --density 700
  python3 compute_inertia.py cylinder --r 0.08 --h 0.04 --density 1100 --axis y

Off-diagonal terms (Ixy, Ixz, Iyz) are zero for a uniform primitive
aligned with its principal axes, which is our case everywhere.
"""
import argparse
import math


def box_inertia(l, w, h, density):
    m = density * l * w * h
    Ixx = m / 12.0 * (w * w + h * h)
    Iyy = m / 12.0 * (l * l + h * h)
    Izz = m / 12.0 * (l * l + w * w)
    return m, Ixx, Iyy, Izz


def cylinder_inertia(r, h, density, axis="z"):
    m = density * math.pi * r * r * h
    I_axial = 0.5 * m * r * r
    I_trans = m / 12.0 * (3.0 * r * r + h * h)
    # Assign to principal axes based on symmetry axis orientation
    if axis == "z":
        Ixx, Iyy, Izz = I_trans, I_trans, I_axial
    elif axis == "y":
        Ixx, Iyy, Izz = I_trans, I_axial, I_trans
    elif axis == "x":
        Ixx, Iyy, Izz = I_axial, I_trans, I_trans
    else:
        raise ValueError(f"axis must be x|y|z, got {axis}")
    return m, Ixx, Iyy, Izz


def emit_urdf(name, m, Ixx, Iyy, Izz, origin=(0, 0, 0)):
    ox, oy, oz = origin
    return (
        f"<!-- {name}: mass={m:.4f} kg -->\n"
        f"<inertial>\n"
        f'  <origin xyz="{ox} {oy} {oz}" rpy="0 0 0"/>\n'
        f'  <mass value="{m:.6f}"/>\n'
        f'  <inertia ixx="{Ixx:.6e}" ixy="0" ixz="0"'
        f' iyy="{Iyy:.6e}" iyz="0" izz="{Izz:.6e}"/>\n'
        f"</inertial>"
    )


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="shape", required=True)

    pb = sub.add_parser("box")
    pb.add_argument("--l", type=float, required=True, help="length along x (m)")
    pb.add_argument("--w", type=float, required=True, help="width along y (m)")
    pb.add_argument("--h", type=float, required=True, help="height along z (m)")
    pb.add_argument("--density", type=float, required=True, help="kg/m^3")
    pb.add_argument("--name", default="box")

    pc = sub.add_parser("cylinder")
    pc.add_argument("--r", type=float, required=True, help="radius (m)")
    pc.add_argument("--h", type=float, required=True, help="length along symmetry axis (m)")
    pc.add_argument("--density", type=float, required=True, help="kg/m^3")
    pc.add_argument("--axis", choices=["x", "y", "z"], default="z")
    pc.add_argument("--name", default="cylinder")

    args = p.parse_args()
    if args.shape == "box":
        m, Ixx, Iyy, Izz = box_inertia(args.l, args.w, args.h, args.density)
    else:
        m, Ixx, Iyy, Izz = cylinder_inertia(args.r, args.h, args.density, args.axis)
    print(emit_urdf(args.name, m, Ixx, Iyy, Izz))


if __name__ == "__main__":
    main()
