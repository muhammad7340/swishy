#!/usr/bin/env python3
"""Render the README's technical figures from a recorded patrol.

Record a bag while a patrol runs on the saved map, then render:

    scripts/swishy.sh --map --mode patrol          # or start the stack by hand
    ros2 bag record -o patrol_bag /map /global_costmap/costmap \
        /local_costmap/costmap /scan /plan /odom /tf /tf_static \
        /left_eye/image /right_eye/image /joint_states
    python3 scripts/make_readme_figures.py patrol_bag assets/

Figures carry no text, so they scale cleanly; the README captions name the
colors. Colors are the first slots of the reference categorical palette:
blue #2a78d6, orange #eb6834, aqua #1baf7a, yellow #eda100, magenta #e87ba4.

  map_trajectory.png   saved map + the path Swishy swam + patrol stations
  costmap_plan.png     global costmap + Nav2's planned path + local window
  lidar_scan.png       one 360 deg LiDAR scan cast from the robot
  showcase_routes.png  showcase routes + the safety envelope around obstacles
  eye_cameras.png      left eye | both fields of view from above | right eye
  tail_wave.png        tail joint angles: the body wave head -> tail
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle, Wedge  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
import rosbag2_py  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "src", "fishy_fish_navigation_py"))
from fishy_fish_navigation_py.navigation_patterns import (  # noqa: E402
    circle, figure_eight, s_curve)

BLUE, ORANGE, AQUA, YELLOW, MAGENTA = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"
SURFACE, UNKNOWN, WALL = "#fcfcfb", "#e7e4dc", "#1f2a37"
RAMP = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95"]  # blue 100..600
STATIONS = [(1.45, -1.45), (1.45, 1.45), (-1.45, 1.45), (-1.45, -1.45)]
VIEW = (-2.0, 2.0, -2.0, 2.0)
EYE_FOV = 1.3962634          # horizontal FOV of each eye camera (swishy.gazebo)


# ---------------------------------------------------------------- bag + TF

def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msgs.setdefault(topic, []).append((stamp * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return msgs


def mat(tr):
    t, q = tr.translation, tr.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [t.x, t.y, t.z]
    return m


class TF:
    """Replays /tf and /tf_static; lookup() returns the 4x4 transform of a
    frame in the map frame at a given bag time."""

    def __init__(self, msgs):
        self.static, self.dynamic = {}, {}
        for _, m in msgs.get("/tf_static", []):
            for t in m.transforms:
                self.static[t.child_frame_id] = (t.header.frame_id, mat(t.transform))
        for stamp, m in msgs.get("/tf", []):
            for t in m.transforms:
                self.dynamic.setdefault(t.child_frame_id, (t.header.frame_id, [], []))
                self.dynamic[t.child_frame_id][1].append(stamp)
                self.dynamic[t.child_frame_id][2].append(mat(t.transform))

    def lookup(self, frame, stamp, target="map"):
        m = np.eye(4)
        while frame != target:
            if frame in self.static:
                parent, t = self.static[frame]
            elif frame in self.dynamic:
                parent, times, mats = self.dynamic[frame]
                i = max(0, min(len(times) - 1, np.searchsorted(times, stamp) - 1))
                t = mats[i]
            else:
                raise KeyError(f"no transform for {frame}")
            m = t @ m
            frame = parent
        return m


def pose2d(m):
    return m[0, 3], m[1, 3], math.atan2(m[1, 0], m[0, 0])


# ---------------------------------------------------------------- drawing

def canvas(size=6.0):
    fig = plt.figure(figsize=(size, size), facecolor=SURFACE)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(SURFACE)
    ax.set_xlim(VIEW[0], VIEW[1])
    ax.set_ylim(VIEW[2], VIEW[3])
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def grid_image(msg, colors):
    """OccupancyGrid -> RGB image; colors(values) maps the int8 grid."""
    g = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    o, r = msg.info.origin.position, msg.info.resolution
    extent = (o.x, o.x + msg.info.width * r, o.y, o.y + msg.info.height * r)
    return colors(g), extent


def hex_rgb(h):
    return [int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)]


def occupancy_colors(g):
    img = np.empty(g.shape + (3,))
    img[:] = hex_rgb(UNKNOWN)
    img[(g >= 0) & (g < 50)] = hex_rgb(SURFACE)
    img[g >= 50] = hex_rgb(WALL)
    return img


def cost_colors(g):
    # Inflation (1-98) on the four lightest blue steps so the free corridors
    # stay readable; inscribed (99) one step darker; lethal (100) as walls.
    img = np.empty(g.shape + (3,))
    img[:] = hex_rgb(UNKNOWN)
    img[g == 0] = hex_rgb(SURFACE)
    light = RAMP[:4]
    for i, lo in enumerate(np.linspace(1, 99, len(light) + 1)[:-1]):
        img[(g >= lo) & (g < 99)] = hex_rgb(light[i])
    img[g == 99] = hex_rgb(RAMP[4])
    img[g == 100] = hex_rgb(WALL)
    return img


def draw_map(ax, msg, colors=occupancy_colors, alpha=1.0):
    img, extent = grid_image(msg, colors)
    ax.imshow(img, origin="lower", extent=extent, interpolation="nearest", alpha=alpha, zorder=0)


def draw_robot(ax, x, y, yaw, color=AQUA, size=0.16, z=10):
    """Swishy seen from above: a fish body with a forked tail, nose along yaw."""
    t = np.linspace(0, 2 * np.pi, 40)
    body = np.c_[0.55 * size * np.cos(t) + 0.15 * size, 0.24 * size * np.sin(t)]
    tail = np.array([[-0.35 * size, 0], [-0.8 * size, 0.3 * size], [-0.65 * size, 0], [-0.8 * size, -0.3 * size]])
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, s], [-s, c]])
    for pts in (tail, body):
        ax.add_patch(Polygon(pts @ rot + [x, y], closed=True, fc=color, ec=SURFACE, lw=1.5, zorder=z))


def save(fig, out, name, crop=None):
    path = os.path.join(out, name)
    fig.savefig(path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("wrote", path)


def latest(msgs, topic, before=None):
    items = msgs.get(topic, [])
    if before is not None:
        items = [i for i in items if i[0] <= before] or items[:1]
    return items[-1]


def nearest(msgs, topic, stamp):
    items = msgs.get(topic, [])
    return min(items, key=lambda i: abs(i[0] - stamp))


# ---------------------------------------------------------------- figures

def fig_map_trajectory(msgs, tf, out):
    fig, ax = canvas()
    draw_map(ax, latest(msgs, "/map")[1])
    stamps = [s for s, _ in msgs["/odom"]][::3]
    path = np.array([pose2d(tf.lookup("base_footprint", s))[:2] for s in stamps])
    ax.plot(path[:, 0], path[:, 1], color=BLUE, lw=2, solid_capstyle="round", zorder=5)
    for x, y in STATIONS:
        ax.add_patch(Circle((x, y), 0.07, fc=ORANGE, ec=SURFACE, lw=2, zorder=6))
    draw_robot(ax, *pose2d(tf.lookup("base_footprint", stamps[0])))
    save(fig, out, "map_trajectory.png")


def fig_costmap(msgs, tf, out, stamp):
    fig, ax = canvas()
    draw_map(ax, nearest(msgs, "/global_costmap/costmap", stamp)[1], cost_colors)
    _, plan = nearest(msgs, "/plan", stamp)
    p = np.array([[q.pose.position.x, q.pose.position.y] for q in plan.poses])
    if len(p):
        ax.plot(p[:, 0], p[:, 1], color=ORANGE, lw=2.5, solid_capstyle="round", zorder=5)
    x, y, yaw = pose2d(tf.lookup("base_footprint", stamp))
    local = nearest(msgs, "/local_costmap/costmap", stamp)[1].info
    ax.add_patch(Rectangle((local.origin.position.x, local.origin.position.y),
                           local.width * local.resolution, local.height * local.resolution,
                           fill=False, ec=YELLOW, lw=2, ls=(0, (6, 4)), zorder=6))
    ax.add_patch(Circle((x, y), 0.22, fill=False, ec=AQUA, lw=2, zorder=7))   # robot_radius
    draw_robot(ax, x, y, yaw)
    save(fig, out, "costmap_plan.png")


def fig_lidar(msgs, tf, out, stamp):
    fig, ax = canvas()
    draw_map(ax, latest(msgs, "/map")[1], alpha=0.35)
    s_stamp, scan = nearest(msgs, "/scan", stamp)
    sx, sy, syaw = pose2d(tf.lookup(scan.header.frame_id, s_stamp))
    a = syaw + scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    r = np.array(scan.ranges)
    ok = (r > scan.range_min) & (r < scan.range_max)
    hx, hy = sx + r[ok] * np.cos(a[ok]), sy + r[ok] * np.sin(a[ok])
    for x, y in zip(hx[::2], hy[::2]):
        ax.plot([sx, x], [sy, y], color=BLUE, lw=0.6, alpha=0.35, zorder=3)
    ax.scatter(hx, hy, s=9, color=ORANGE, zorder=5, linewidths=0)
    draw_robot(ax, *pose2d(tf.lookup("base_footprint", s_stamp)))
    save(fig, out, "lidar_scan.png")


def fig_routes(msgs, out):
    fig, ax = canvas()
    draw_map(ax, latest(msgs, "/map")[1], alpha=0.6)
    prof = yaml.safe_load(open(os.path.join(
        ROOT, "src", "fishy_fish_navigation_py", "config", "showcase_profiles.yaml")))
    p = prof["showcase_navigation"]["ros__parameters"]
    clearance = p["robot_radius"] + p["safety_clearance"]
    c = p["obstacle_centers"]
    for i, rad in enumerate(p["obstacle_radii"]):
        ax.add_patch(Circle((c[2 * i], c[2 * i + 1]), rad + clearance, fc=MAGENTA, alpha=0.1, zorder=2))
        ax.add_patch(Circle((c[2 * i], c[2 * i + 1]), rad + clearance, fill=False, ec=MAGENTA, lw=1.5, zorder=2))
    n = p["path_samples"]

    def line(pts, color, **kw):
        a = np.array([[q.x, q.y] for q in pts])
        ax.plot(a[:, 0], a[:, 1], color=color, lw=2.2, solid_capstyle="round", zorder=5, **kw)
    line(figure_eight(0.0, -0.65, 1.15, 0.42, n), BLUE)
    line(circle(1.0, 0.8, 0.65, n), ORANGE)
    line(circle(0.0, -0.7, 0.38, n), AQUA)
    line(s_curve(0.0, -0.7, 1.1, 0.28, n), AQUA)
    sq = np.array(STATIONS + STATIONS[:1])
    ax.plot(sq[:, 0], sq[:, 1], color=YELLOW, lw=2.2, ls=(0, (7, 5)), zorder=4)
    for x, y in STATIONS:
        ax.add_patch(Circle((x, y), 0.07, fc=YELLOW, ec=SURFACE, lw=2, zorder=6))
    draw_robot(ax, 0.0, 0.0, 0.0)
    save(fig, out, "showcase_routes.png")


def image_rgb(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
    return a[:, :, ::-1] if msg.encoding.lower().startswith("bgr") else a


def fig_eyes(msgs, tf, out, stamp):
    _, left = nearest(msgs, "/left_eye/image", stamp)
    _, right = nearest(msgs, "/right_eye/image", stamp)
    fig = plt.figure(figsize=(12, 3.0), facecolor=SURFACE)
    w = 0.32
    for i, (img, col) in enumerate([(image_rgb(left), BLUE), (None, None), (image_rgb(right), ORANGE)]):
        ax = fig.add_axes([0.005 + i * (w + 0.0175), 0.02, w, 0.96])
        if img is not None:
            ax.imshow(img)
            ax.set_xticks([])
            ax.set_yticks([])
            for s in ax.spines.values():
                s.set_color(col)
                s.set_linewidth(4)
            continue
        x, y, yaw = pose2d(tf.lookup("base_footprint", stamp))
        ax.set_facecolor(SURFACE)
        ax.axis("off")
        ax.set_aspect("equal")
        span = 1.2
        ax.set_xlim(x - span * 4 / 3, x + span * 4 / 3)
        ax.set_ylim(y - span, y + span)
        draw_map(ax, latest(msgs, "/map")[1], alpha=0.55)
        for side, col in (("left", BLUE), ("right", ORANGE)):
            ex, ey, eyaw = pose2d(tf.lookup(f"{side}_eye_camera_link", stamp))
            d = math.degrees(eyaw)
            h = math.degrees(EYE_FOV) / 2
            ax.add_patch(Wedge((ex, ey), 1.1, d - h, d + h, fc=col, alpha=0.22, ec=col, lw=2, zorder=4))
        draw_robot(ax, x, y, yaw, size=0.2)
    save(fig, out, "eye_cameras.png")


def fig_tail(msgs, out):
    # sim time from the header: the joints move smoothly in sim time even when
    # a busy machine makes the bag's wall-clock stamps uneven
    by_stamp = {m.header.stamp.sec + m.header.stamp.nanosec * 1e-9: dict(zip(m.name, m.position))
                for _, m in msgs["/joint_states"]}
    js = sorted(by_stamp.items())
    # the 3 s window where the tail swings most (Swishy at cruising speed)
    t0 = max((s for s, _ in js[::5]),
             key=lambda a: np.std([p["joint_3"] for s, p in js if a <= s <= a + 3.0] or [0]))
    rows = [(s - t0, p) for s, p in js if t0 <= s <= t0 + 3.0]
    t = np.array([r[0] for r in rows])
    fig = plt.figure(figsize=(9, 2.6), facecolor=SURFACE)
    ax = fig.add_axes([0.06, 0.14, 0.92, 0.8])
    ax.set_facecolor(SURFACE)
    for name, col in (("joint_1", BLUE), ("joint_2", ORANGE), ("joint_3", AQUA)):
        ax.plot(t, [math.degrees(r[1][name]) for r in rows], color=col, lw=2)
    ax.axhline(0, color="#d6d3cb", lw=1, zorder=0)
    ax.grid(axis="y", color="#ece9e3", lw=0.8)
    ax.tick_params(colors="#52514e", labelsize=9)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color("#d6d3cb")
    save(fig, out, "tail_wave.png")


def main():
    bag, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    msgs = read_bag(bag)
    tf = TF(msgs)

    # costmap: the moment Nav2 had its longest plan, so the whole path shows
    def plan_length(m):
        p = [(q.pose.position.x, q.pose.position.y) for q in m.poses]
        return sum(math.dist(a, b) for a, b in zip(p, p[1:]))
    long_plan = max(msgs["/plan"], key=lambda i: plan_length(i[1]))[0]
    # LiDAR and eyes: the moment the robot was closest to the open middle
    middle = min((s for s, _ in msgs["/odom"][::10]),
                 key=lambda s: math.dist(pose2d(tf.lookup("base_footprint", s))[:2], (0.3, -0.4)))

    fig_map_trajectory(msgs, tf, out)
    fig_costmap(msgs, tf, out, long_plan)
    fig_lidar(msgs, tf, out, middle)
    fig_routes(msgs, out)
    fig_eyes(msgs, tf, out, middle)
    if "/joint_states" in msgs:
        fig_tail(msgs, out)


if __name__ == "__main__":
    main()
