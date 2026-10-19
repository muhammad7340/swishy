import math

from fishy_fish_navigation_py.navigation_patterns import AquariumSafety
from fishy_fish_navigation_py.navigation_patterns import circle
from fishy_fish_navigation_py.navigation_patterns import CircularObstacle
from fishy_fish_navigation_py.navigation_patterns import figure_eight
from fishy_fish_navigation_py.navigation_patterns import PathPoint
from fishy_fish_navigation_py.navigation_patterns import s_curve


def aquarium_safety():
    return AquariumSafety(
        half_width=2.0,
        half_height=2.0,
        robot_radius=0.22,
        clearance=0.12,
        obstacles=[
            CircularObstacle(1.0, 0.8, 0.2),
            CircularObstacle(-1.3, 0.85, math.hypot(0.15, 0.15)),
        ],
    )


def test_showcase_patterns_fit_aquarium():
    safety = aquarium_safety()
    paths = [
        figure_eight(0.0, -0.65, 1.15, 0.42, 140),
        circle(1.0, 0.8, 0.65, 100),
    ]

    for path in paths:
        safety.validate(path, 'showcase')


def test_dance_patterns_fit_aquarium():
    safety = aquarium_safety()
    paths = [
        circle(0.0, -0.7, 0.38, 80, loops=2),
        s_curve(0.0, -0.7, 1.1, 0.28, 100),
        figure_eight(0.0, -0.7, 0.55, 0.28, 100),
    ]

    for path in paths:
        safety.validate(path, 'dance')


def test_rejects_wall_and_obstacle_points():
    safety = aquarium_safety()

    assert not safety.is_safe(PathPoint(1.9, 0.0))
    assert not safety.is_safe(PathPoint(1.0, 0.8))
    assert safety.is_safe(PathPoint(0.0, 0.0))
