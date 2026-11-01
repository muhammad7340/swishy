#!/usr/bin/env python3
"""Generate the flat, finely tessellated water-surface mesh for the aquarium.

The Gerstner-wave vertex shader can only move vertices that exist, so the
surface needs a dense grid. VRX's own mesh is 60 x 60 m; scaling it down would
also scale (distort) the wavelengths, so we generate a tank-sized grid instead.

Usage:
    python3 scripts/make_water_grid.py [out.obj] [size_x] [size_y] [cells_per_m]

Defaults match the fish tank interior (inside the glass, 3.73 x 3.635 m):
    src/fishy_fish_navigation/models/aquarium_water/meshes/water_grid.obj 3.72 3.62 25
"""
import sys

DEFAULT_OUT = "src/fishy_fish_navigation/models/aquarium_water/meshes/water_grid.obj"


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUT
    lx = float(sys.argv[2]) if len(sys.argv) > 2 else 3.72
    ly = float(sys.argv[3]) if len(sys.argv) > 3 else 3.62
    density = float(sys.argv[4]) if len(sys.argv) > 4 else 25.0  # 4 cm spacing

    nx = max(1, round(lx * density))
    ny = max(1, round(ly * density))

    with open(out, "w") as f:
        f.write(f"# aquarium water grid {lx} x {ly} m, {nx} x {ny} cells\n")
        for j in range(ny + 1):
            for i in range(nx + 1):
                f.write(f"v {-lx / 2 + lx * i / nx:.4f} {-ly / 2 + ly * j / ny:.4f} 0\n")
        for j in range(ny + 1):
            for i in range(nx + 1):
                f.write(f"vt {i / nx:.4f} {j / ny:.4f}\n")
        f.write("vn 0 0 1\n")
        for j in range(ny):
            for i in range(nx):
                a = j * (nx + 1) + i + 1          # OBJ indices are 1-based
                b, c, d = a + 1, a + nx + 2, a + nx + 1
                # counter-clockwise seen from above -> faces point +Z
                f.write(f"f {a}/{a}/1 {b}/{b}/1 {c}/{c}/1\n")
                f.write(f"f {a}/{a}/1 {c}/{c}/1 {d}/{d}/1\n")
    print(f"wrote {out}: {(nx + 1) * (ny + 1)} vertices, {2 * nx * ny} triangles")


if __name__ == "__main__":
    main()
