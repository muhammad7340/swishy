#!/usr/bin/env python3
"""Generate the underwater-effect assets for the aquarium_water model.

All meshes are in the aquarium_water model frame (origin = water surface,
tank interior 3.73 x 3.635 m, sand ~0.6-0.78 m below the surface).

  meshes/fog_shells.obj      nested inward-facing spheres; the fog shader moves
                             them onto the camera (UnderwaterFog_*.glsl)
  meshes/god_rays.obj        tapered crossed strips from the surface to the sand
  meshes/bubbles.obj         bubble columns; the bubble shader makes them rise
  meshes/pump_bubbles.obj    the pump's chain of small, evenly spaced bubbles
  meshes/sand_caustics.obj   copy of the sand quad lifted 4 mm, drawn with the
                             animated caustics shader (SandCaustics_*.glsl)

It also writes aquarium_water_deep, the same water 0.7 m deeper for the glass
tank (fish_world_glass.sdf, surface at z = 2.3): its own shafts, bubbles and
caustics, a model.sdf derived from aquarium_water's, and symlinks to the shared
shaders, textures, surface grid and fog shells.

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
WATER_DEEP = os.path.join(PKG, "aquarium_water_deep")
WATER_Z, DEEP_WATER_Z = 1.6, 2.3   # world z of each model's origin (the surface)

TANK_X, TANK_Y = 3.72, 3.62
SURFACE_Z = -0.03   # just below the wave troughs
SAND_Z = -0.75
DEEP_SAND_Z = SAND_Z - (DEEP_WATER_Z - WATER_Z)
# Outlet flange of models/aquarium_pump in the world (pump at (-1.3, 0.85,
# 0.871), outlet 0.168 m along +y and 0.405 m up): the pump's bubble stream.
PUMP_OUTLET = (-1.3, 1.018, 1.276)


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


def god_rays(n=7, seed=3, out=WATER, sand_z=SAND_Z):
    rng = random.Random(seed)
    obj = Obj()
    lean = (0.22, 0.12)   # sun direction: horizontal shift per metre of depth
    depth = SURFACE_Z - sand_z
    for k in range(n):
        x0 = rng.uniform(-1.5, 1.5)
        y0 = rng.uniform(-1.4, 1.4)
        w_top, w_bot = rng.uniform(0.12, 0.2), rng.uniform(0.3, 0.45)
        top = (x0, y0, SURFACE_Z)
        bot = (x0 + lean[0] * depth, y0 + lean[1] * depth, sand_z)
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
    obj.write(os.path.join(out, "meshes", "god_rays.obj"), f"{n} light shafts")



def bubbles(per_column=14, radius=0.02, seed=7, out=WATER, sand_z=SAND_Z):
    # Columns sit where the old particle emitters were. Each bubble's vertices
    # share one uv: u = phase offset (0..1), v = speed factor. The shader
    # recovers the centre as vertex - normal * radius and lets it rise to topZ.
    rng = random.Random(seed)
    obj = Obj()
    columns = [(1.55, 1.35), (-1.55, -1.45), (0.4, -1.6)]
    for cx, cy in columns:
        for _ in range(per_column):
            c = (cx + rng.uniform(-0.06, 0.06), cy + rng.uniform(-0.06, 0.06), sand_z + 0.03)
            sphere(obj, c, radius, 18, 12, (rng.random(), rng.uniform(0.8, 1.25)))
    obj.write(os.path.join(out, "meshes", "bubbles.obj"),
              f"{len(columns)} bubble columns x {per_column}, radius {radius} m")


def pump_bubbles(n=18, radius=0.01, seed=11, out=WATER, water_z=WATER_Z):
    # An air pump releases small bubbles one after another: evenly spaced
    # phases and one speed give a steady single-file chain, not a plume.
    # Drawn by its own visual, whose radius uniform matches this mesh.
    rng = random.Random(seed)
    obj = Obj()
    px, py, pz = PUMP_OUTLET[0], PUMP_OUTLET[1] + 0.05, PUMP_OUTLET[2] - water_z
    for k in range(n):
        c = (px + rng.uniform(-0.004, 0.004), py + rng.uniform(-0.004, 0.004), pz)
        sphere(obj, c, radius, 12, 8, (k / n + rng.uniform(-0.01, 0.01), rng.uniform(0.97, 1.03)))
    obj.write(os.path.join(out, "meshes", "pump_bubbles.obj"),
              f"pump bubble chain: {n} bubbles, radius {radius} m")


def sand_caustics(lift=0.004, out=WATER, water_z=WATER_Z):
    # The sand in models/model.dae is one slightly tilted quad (node "Plane").
    # World-frame corners, read from the .dae; the water model sits at
    # (0, -0.05, 1.6) in fish_world.sdf.
    corners = np.array([[-1.8884, -1.9912, 0.8231], [1.8856, -1.9912, 1.0020],
                        [1.8841, 1.7870, 1.0344], [-1.8900, 1.7870, 0.8556]])
    corners -= (0.0, -0.05, water_z)
    n = np.cross(corners[1] - corners[0], corners[2] - corners[0])
    n /= np.linalg.norm(n)
    corners += lift * n
    obj = Obj()
    a, b, c, d = (obj.vert(tuple(p), (0, 0), tuple(n)) for p in corners)
    obj.tri(a, b, c)
    obj.tri(a, c, d)
    obj.write(os.path.join(out, "meshes", "sand_caustics.obj"), "sand caustics overlay")


def deep_water():
    """aquarium_water_deep: the same water, DEEP_WATER_Z - WATER_Z deeper."""
    extra = DEEP_WATER_Z - WATER_Z
    os.makedirs(os.path.join(WATER_DEEP, "meshes"), exist_ok=True)
    links = {"materials": "../aquarium_water/materials",
             "meshes/water_grid.obj": "../../aquarium_water/meshes/water_grid.obj",
             "meshes/fog_shells.obj": "../../aquarium_water/meshes/fog_shells.obj"}
    for name, target in links.items():
        path = os.path.join(WATER_DEEP, name)
        if os.path.lexists(path):
            os.remove(path)
        os.symlink(target, path)
    god_rays(out=WATER_DEEP, sand_z=DEEP_SAND_Z)
    bubbles(out=WATER_DEEP, sand_z=DEEP_SAND_Z)
    pump_bubbles(out=WATER_DEEP, water_z=DEEP_WATER_Z)
    sand_caustics(out=WATER_DEEP, water_z=DEEP_WATER_Z)

    s = open(os.path.join(WATER, "model.sdf")).read()
    replace = [
        ('<model name="aquarium_water">', '<model name="aquarium_water_deep">'),
        # volume: top stays 3 cm under the surface, bottom follows the sand
        ("<pose>0 0 -0.39 0 0 0</pose>", f"<pose>0 0 {-0.39 - extra / 2:.2f} 0 0 0</pose>"),
        ("<box><size>3.72 3.62 0.72</size></box>", f"<box><size>3.72 3.62 {0.72 + extra:.2f}</size></box>"),
    ]
    for old, new in replace:
        assert s.count(old) == 1, old
        s = s.replace(old, new)
    note = (f"<!-- GENERATED by scripts/make_aquarium_effects.py from aquarium_water/model.sdf:\n"
            f"     the same water {extra:.1f} m deeper, for fish_world_glass.sdf (surface at\n"
            f"     z = {DEEP_WATER_Z}). Edit aquarium_water and re-run the script. -->\n")
    s = s.replace("<sdf", note + "<sdf", 1)
    open(os.path.join(WATER_DEEP, "model.sdf"), "w").write(s)
    open(os.path.join(WATER_DEEP, "model.config"), "w").write(
        open(os.path.join(WATER, "model.config")).read()
        .replace("<name>aquarium_water</name>", "<name>aquarium_water_deep</name>")
        .replace("Visual-only aquarium water", f"Visual-only aquarium water, {extra:.1f} m deeper,"))
    print(f"wrote {os.path.relpath(WATER_DEEP)}")


if __name__ == "__main__":
    fog_shells()
    god_rays()
    bubbles()
    pump_bubbles()
    sand_caustics()
    deep_water()
