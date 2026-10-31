#!/usr/bin/env python3
"""Generate the underwater-effect assets for the aquarium_water model.

All meshes are in the aquarium_water model frame (origin = water surface,
tank interior 3.9 x 3.8 m, sand ~0.6-0.78 m below the surface).

  meshes/fog_shells.obj      nested inward-facing spheres; the fog shader moves
                             them onto the camera (UnderwaterFog_*.glsl)
  meshes/god_rays.obj        tapered crossed strips from the surface to the sand
  meshes/bubbles.obj         bubble columns; the bubble shader makes them rise
  meshes/sand_caustics.obj   copy of the sand quad lifted 4 mm, drawn with the
                             animated caustics shader (SandCaustics_*.glsl)

Usage:
    python3 scripts/make_aquarium_effects.py
"""
import math
import os
import random

import numpy as np

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "src", "fishy_fish_navigation", "models")
WATER = os.path.join(PKG, "aquarium_water")

TANK_X, TANK_Y = 3.9, 3.8
SURFACE_Z = -0.03   # just below the wave troughs
SAND_Z = -0.75


class Obj:
    def __init__(self):
        self.v, self.vt, self.vn, self.f = [], [], [], []

    def vert(self, p, t, n):
        self.v.append(p)
        self.vt.append(t)
        self.vn.append(n)
        return len(self.v)

    def tri(self, a, b, c):
        self.f.append((a, b, c))

    def write(self, path, header):
        with open(path, "w") as f:
            f.write(f"# {header}\n")
            for p in self.v:
                f.write("v {:.5f} {:.5f} {:.5f}\n".format(*p))
            for t in self.vt:
                f.write("vt {:.5f} {:.5f}\n".format(*t))
            for n in self.vn:
                f.write("vn {:.5f} {:.5f} {:.5f}\n".format(*n))
            for tri in self.f:
                f.write("f " + " ".join(f"{i}/{i}/{i}" for i in tri) + "\n")
        print(f"wrote {os.path.relpath(path)}: {len(self.v)} verts, {len(self.f)} tris")


def sphere(obj, center, radius, n_lon, n_lat, uv, inward=False):
    rows = []
    for j in range(n_lat + 1):
        th = math.pi * j / n_lat
        row = []
        for i in range(n_lon + 1):
            ph = 2 * math.pi * i / n_lon
            n = (math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th))
            p = tuple(c + radius * k for c, k in zip(center, n))
            row.append(obj.vert(p, uv, tuple(-k for k in n) if inward else n))
        rows.append(row)
    for j in range(n_lat):
        for i in range(n_lon):
            a, b = rows[j][i], rows[j][i + 1]
            c, d = rows[j + 1][i + 1], rows[j + 1][i]
            # outward-facing CCW is (a, d, c) / (a, c, b)
            if inward:
                obj.tri(a, c, d)
                obj.tri(a, b, c)
            else:
                obj.tri(a, d, c)
                obj.tri(a, c, b)


def fog_shells(spacing=0.1, r_max=4.0):
    # Outermost shell first: the camera is always at the centre, so this is
    # back-to-front order for every view.
    obj = Obj()
    n = int(round(r_max / spacing))
    for k in range(n, 0, -1):
        sphere(obj, (0, 0, 0), k * spacing, 32, 16, (0, 0), inward=True)
    obj.write(os.path.join(WATER, "meshes", "fog_shells.obj"),
              f"{n} inward fog shells, spacing {spacing} m")


def god_rays(n=7, seed=3):
    rng = random.Random(seed)
    obj = Obj()
    lean = (0.22, 0.12)   # sun direction: horizontal shift per metre of depth
    depth = SURFACE_Z - SAND_Z
    for k in range(n):
        x0 = rng.uniform(-1.5, 1.5)
        y0 = rng.uniform(-1.4, 1.4)
        w_top, w_bot = rng.uniform(0.12, 0.2), rng.uniform(0.3, 0.45)
        top = (x0, y0, SURFACE_Z)
        bot = (x0 + lean[0] * depth, y0 + lean[1] * depth, SAND_Z)
        # two crossed strips so the shaft has width from every side; the
        # integer part of u is the ray index (per-ray sway/flicker seed)
        for ang in (0.0, math.pi / 2):
            d = (math.cos(ang), math.sin(ang), 0.0)
            nrm = (-d[1], d[0], 0.0)
            quad = [
                obj.vert(tuple(t - d[i] * w_top / 2 if i < 2 else t for i, t in enumerate(top)), (k + 0.001, 0), nrm),
                obj.vert(tuple(t + d[i] * w_top / 2 if i < 2 else t for i, t in enumerate(top)), (k + 0.999, 0), nrm),
                obj.vert(tuple(t + d[i] * w_bot / 2 if i < 2 else t for i, t in enumerate(bot)), (k + 0.999, 1), nrm),
                obj.vert(tuple(t - d[i] * w_bot / 2 if i < 2 else t for i, t in enumerate(bot)), (k + 0.001, 1), nrm),
            ]
            a, b, c, e = quad
            for tri in ((a, b, c), (a, c, e), (a, c, b), (a, e, c)):
                obj.tri(*tri)
    obj.write(os.path.join(WATER, "meshes", "god_rays.obj"), f"{n} light shafts")



def bubbles(per_column=14, radius=0.02, seed=7):
    # Columns sit where the old particle emitters were. Each bubble's vertices
    # share one uv: u = phase offset (0..1), v = speed factor. The shader
    # recovers the centre as vertex - normal * radius.
    rng = random.Random(seed)
    obj = Obj()
    columns = [(1.55, 1.35), (-1.55, -1.45), (0.4, -1.6)]
    for cx, cy in columns:
        for _ in range(per_column):
            c = (cx + rng.uniform(-0.06, 0.06), cy + rng.uniform(-0.06, 0.06), SAND_Z + 0.03)
            sphere(obj, c, radius, 18, 12, (rng.random(), rng.uniform(0.8, 1.25)))
    obj.write(os.path.join(WATER, "meshes", "bubbles.obj"),
              f"{len(columns)} bubble columns x {per_column}, radius {radius} m")


def sand_caustics(lift=0.004):
    # The sand in models/model.dae is one slightly tilted quad (node "Plane").
    # World-frame corners, read from the .dae; the water model sits at
    # (0, -0.05, 1.6) in fish_world.sdf.
    corners = np.array([[-1.8884, -1.9912, 0.8231], [1.8856, -1.9912, 1.0020],
                        [1.8841, 1.7870, 1.0344], [-1.8900, 1.7870, 0.8556]])
    corners -= (0.0, -0.05, 1.6)
    n = np.cross(corners[1] - corners[0], corners[2] - corners[0])
    n /= np.linalg.norm(n)
    corners += lift * n
    obj = Obj()
    a, b, c, d = (obj.vert(tuple(p), (0, 0), tuple(n)) for p in corners)
    obj.tri(a, b, c)
    obj.tri(a, c, d)
    obj.write(os.path.join(WATER, "meshes", "sand_caustics.obj"), "sand caustics overlay")


if __name__ == "__main__":
    fog_shells()
    god_rays()
    bubbles()
    sand_caustics()
