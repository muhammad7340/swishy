#!/usr/bin/env python3
"""Place the aquarium decor in fish_world.sdf and check it against the routes.

The layout lives in the tables below. Running the script:
  1. computes each item's z from the tilted sand (one flat quad in model.dae),
  2. checks that plant tops stay under the water and that hard decor stays
     inside the glass and clear of every showcase route and patrol station,
  3. rewrites the block between the BEGIN/END markers in fish_world.sdf.

Usage:
    python3 scripts/layout_decor.py          # check and write
    python3 scripts/layout_decor.py --check  # check only
"""
import math
import os
import re
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
WORLD = os.path.join(ROOT, "src", "fishy_fish_navigation", "worlds", "fish_world.sdf")
sys.path.insert(0, os.path.join(ROOT, "src", "fishy_fish_navigation_py"))
from fishy_fish_navigation_py.navigation_patterns import (  # noqa: E402
    AquariumSafety, CircularObstacle, PathPoint, circle, figure_eight, s_curve)

WATER_Z = 1.6
PLANT_TOP_LIMIT = WATER_Z - 0.05
# Inner faces of the tank glass (Cube mesh in model.dae); the gpu_lidar and
# the camera see these, not the collision walls at +/-2.0.
GLASS_X = 1.865
GLASS_Y_MIN, GLASS_Y_MAX = -1.865, 1.77

# Model heights (m) and horizontal radius, from the model.sdf headers.
HEIGHT = {
    "kelp_01": 0.564, "kelp_02": 0.602, "kelp_03": 0.619, "kelp_04": 0.639,
    "grass_patch": 0.183, "grass_patch_2": 0.200, "plant_bush": 0.152,
    "cactus_tall": 0.500, "cactus_small": 0.280,
}
RADIUS = {
    "cactus_tall": 0.185, "cactus_small": 0.108, "rock_moss": 0.19,
    "driftwood": 0.20, "ocean_rock_01": 0.105, "ocean_rock_03": 0.075,
    "ocean_rock_05": 0.07, "ocean_rock_06": 0.07,
}


def floor_z(x, y):
    """Sand height: the sand is one slightly tilted quad (0.82 - 1.03 m)."""
    return 0.930 + 0.0474 * x + 0.0086 * y


# (model, x, y, yaw, bury) -- bury sinks the base into the sand where the
# water is shallow (east side) so tall kelp stays under the surface.
KELP = [
    # back row, in front of the backdrop
    ("kelp_04", -1.70, 1.58, 0.3, 0.0),
    ("kelp_03", -1.25, 1.58, 1.9, 0.0),
    ("kelp_02", -0.80, 1.58, 4.1, 0.0),
    ("kelp_03", -0.30, 1.58, 2.6, 0.0),
    ("kelp_02", 0.20, 1.58, 5.2, 0.01),
    ("kelp_01", 0.65, 1.58, 0.9, 0.0),
    ("kelp_01", 1.25, 1.58, 3.3, 0.02),
    # west glass (clear of the coral at y -0.30 .. 0.17)
    ("kelp_04", -1.70, -1.00, 2.2, 0.0),
    ("kelp_02", -1.70, -0.55, 0.7, 0.0),
    ("kelp_01", -1.70, 0.40, 3.9, 0.0),
    # east glass, y < 0 only: the orbit around the cylinder uses y > 0
    ("kelp_01", 1.70, -1.10, 1.4, 0.02),
    ("kelp_01", 1.70, -0.40, 4.6, 0.025),
]

# Hard decor. "lidar_r" is the footprint the LiDAR sees at scan height
# (z ~1.15); None means the piece is lower than the beam and the robot's belly,
# so it is not a navigation obstacle.
HARD = [
    # NE corner: cactus with its arms along the corner diagonal
    ("cactus_small", 1.795, 1.700, 0.785, 0.0, 0.07),
    # NW corner: low driftwood along the back glass + a pebble
    ("driftwood", -1.665, 1.700, 1.5708, 0.0, None),
    ("ocean_rock_05", -1.79, 1.48, 1.2, 0.0, None),
    # SW corner
    ("ocean_rock_01", -1.765, -1.765, 0.6, 0.0, 0.095),
    ("ocean_rock_05", -1.60, -1.80, 2.5, 0.0, None),
    # SE corner: cactus with its arms along the corner diagonal + a pebble
    ("cactus_small", 1.795, -1.795, -0.785, 0.0, 0.07),
    ("ocean_rock_06", 1.62, -1.80, 0.4, 0.0, None),
    # focal point
    ("rock_moss", -0.62, 0.66, 0.5, 0.0, None),
    ("cactus_tall", -0.45, 0.76, 0.3, 0.0, None),
    ("ocean_rock_03", -0.47, 0.50, 1.1, 0.0, None),
]
# The focal cluster is one navigation obstacle around its centre.
FOCAL = (-0.55, 0.65, 0.30)

SOFT = [
    # ring around the focal point
    ("grass_patch", -0.85, 0.52, 0.4),
    ("plant_bush", -0.40, 0.36, 2.0),
    ("grass_patch_2", -0.80, 0.86, 4.0),
    # floor carpet
    ("grass_patch", -1.20, -1.20, 0.0),
    ("plant_bush", -0.30, -1.60, 1.1),
    ("grass_patch_2", 0.95, -1.45, 2.3),
    ("grass_patch", 1.30, -0.30, 3.4),
    ("plant_bush", -1.00, 0.35, 4.5),
    ("grass_patch_2", 0.20, 0.30, 5.6),
    ("grass_patch", 1.50, -1.60, 0.7),
    ("grass_patch_2", -1.60, -0.80, 1.8),
    # corner bases
    ("grass_patch", 1.55, 1.68, 2.9),
    ("grass_patch_2", -1.72, 1.50, 4.0),
    ("plant_bush", -1.55, -1.75, 5.1),
    ("grass_patch", 1.78, -1.58, 0.2),
    # front glass
    ("seashell_01", -0.70, -1.78, 0.5),
    ("seashell_02", -0.55, -1.73, 2.4),
    ("seashell_03", 0.95, -1.80, 4.0),
    ("starfish", -0.62, -1.64, 0.4),
]
LILYPADS = [(-1.50, 1.30, 0.3), (-1.20, 1.55, 2.0), (1.30, -1.40, 4.1), (1.55, -1.15, 5.5)]


def route_points():
    """Every showcase route and patrol leg (mirrors showcase_navigation.py)."""
    n = 140
    routes = {
        "figure_eight": figure_eight(0.0, -0.65, 1.15, 0.42, n),
        "orbit": circle(1.0, 0.8, 0.65, n),
        "dance_circle": circle(0.0, -0.7, 0.38, n, loops=2),
        "dance_s": s_curve(0.0, -0.7, 1.1, 0.28, n),
        "dance_eight": figure_eight(0.0, -0.7, 0.55, 0.28, n),
        "home": [PathPoint(0.0, 0.0)],
    }
    stations = [(1.45, -1.45), (1.45, 1.45), (-1.45, 1.45), (-1.45, -1.45)]
    routes["patrol_stations"] = [PathPoint(x, y) for x, y in stations]
    legs = []
    for (ax, ay), (bx, by) in zip(stations, stations[1:] + stations[:1]):
        legs += [PathPoint(ax + (bx - ax) * t / 40, ay + (by - ay) * t / 40) for t in range(41)]
    routes["patrol_legs"] = legs
    return routes


def nav_obstacles():
    obs = [CircularObstacle(*FOCAL)]
    obs += [CircularObstacle(x, y, r) for m, x, y, yaw, bury, r in HARD if r is not None]
    return obs


def check():
    ok = True
    for m, x, y, yaw, bury in KELP:
        top = floor_z(x, y) - bury + HEIGHT[m]
        if top > PLANT_TOP_LIMIT:
            print(f"FAIL {m} at ({x}, {y}): top {top:.3f} > {PLANT_TOP_LIMIT}")
            ok = False
    for m, x, y, yaw, bury, r in HARD:
        if m in HEIGHT and floor_z(x, y) + HEIGHT[m] > PLANT_TOP_LIMIT:
            print(f"FAIL {m} at ({x}, {y}) reaches the surface")
            ok = False
        rr = r if r is not None else 0.0
        if abs(x) + rr > GLASS_X or not (GLASS_Y_MIN <= y - rr and y + rr <= GLASS_Y_MAX):
            print(f"FAIL {m} at ({x}, {y}) footprint r {rr} crosses the glass")
            ok = False
    safety = AquariumSafety(2.0, 2.0, 0.22, 0.12, nav_obstacles())
    for name, pts in route_points().items():
        worst = min(
            (math.hypot(p.x - o.x, p.y - o.y) - o.radius - 0.22 for p in pts for o in safety.obstacles),
            default=99)
        bad = [p for p in pts if not safety.is_safe(p)]
        # patrol stations/legs are Nav2 goals, not validated paths: only the
        # decor obstacles matter, so ignore the +/-1.66 wall envelope there
        if name.startswith("patrol"):
            bad = [p for p in bad if any(
                math.hypot(p.x - o.x, p.y - o.y) < o.radius + 0.34 for o in safety.obstacles)]
        status = "ok" if not bad else f"FAIL ({len(bad)} points)"
        print(f"{name:16s} min gap to decor edge minus robot radius {worst:5.2f} m  {status}")
        ok &= not bad
    return ok


def include(model, x, y, z, yaw, name):
    return (f'<include>\n  <uri>model://{model}</uri>\n  <name>{name}</name>\n'
            f'  <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>\n</include>\n')


def block():
    out, count = [], {}

    def name(m):
        count[m] = count.get(m, 0) + 1
        return f"decor_{m}_{count[m]}"

    out.append("<!-- Back glass backdrop (soft decor) -->\n")
    out.append(include("backdrop", 0.0, GLASS_Y_MAX - 0.005, 0.80, 0.0, "decor_backdrop"))
    out.append("<!-- Kelp: soft decor, hidden from the LiDAR (visibility_flags 8) -->\n")
    out += [include(m, x, y, floor_z(x, y) - bury, yaw, name(m)) for m, x, y, yaw, bury in KELP]
    out.append("<!-- Hard decor: corners and the focal point; mapped by the LiDAR -->\n")
    out += [include(m, x, y, floor_z(x, y) - bury, yaw, name(m)) for m, x, y, yaw, bury, r in HARD]
    out.append("<!-- Soft decor on the sand: grass, ferns, shells, starfish -->\n")
    out += [include(m, x, y, floor_z(x, y) + (0.002 if m.startswith(("seashell", "starfish")) else -0.01),
                    yaw, name(m)) for m, x, y, yaw in SOFT]
    out.append("<!-- Lily pads floating on the surface -->\n")
    out += [include("lilypad", x, y, WATER_Z + 0.015, yaw, name("lilypad")) for x, y, yaw in LILYPADS]
    return "".join(out)


BEGIN = "<!-- BEGIN aquarium decor (generated by scripts/layout_decor.py) -->\n"
END = "<!-- END aquarium decor -->\n"
ANCHOR = "<include>\n  <uri>model://aquarium_water</uri>\n  <pose>0 -0.05 1.6 0 0 0</pose>\n</include>\n"


def write():
    s = open(WORLD).read()
    new = BEGIN + block() + END
    if BEGIN in s:
        s = re.sub(re.escape(BEGIN) + ".*?" + re.escape(END), lambda _: new, s, flags=re.S)
    else:
        if ANCHOR not in s:
            sys.exit("aquarium_water include not found in fish_world.sdf")
        s = s.replace(ANCHOR, ANCHOR + "\n" + new)
    open(WORLD, "w").write(s)
    print(f"wrote {sum(1 for _ in re.finditer('<include>', new))} decor includes to {os.path.relpath(WORLD)}")


if __name__ == "__main__":
    good = check()
    print("obstacles for showcase_profiles.yaml:",
          [(round(o.x, 3), round(o.y, 3), o.radius) for o in nav_obstacles()])
    if not good:
        sys.exit("layout check failed")
    if "--check" not in sys.argv:
        write()
