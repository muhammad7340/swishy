import math
import os

from fishy_fish_navigation_py.navigation_patterns import AquariumSafety
from fishy_fish_navigation_py.navigation_patterns import circle
from fishy_fish_navigation_py.navigation_patterns import CircularObstacle
from fishy_fish_navigation_py.navigation_patterns import figure_eight
from fishy_fish_navigation_py.navigation_patterns import PathPoint
from fishy_fish_navigation_py.navigation_patterns import s_curve
import yaml

PROFILES = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'config', 'showcase_profiles.yaml',
)


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


def test_configured_routes_clear_every_obstacle():
    # Same routes as showcase_navigation.py, checked against the obstacle
    # list in showcase_profiles.yaml (aquarium decor included).
    with open(PROFILES) as f:
        params = yaml.safe_load(f)['showcase_navigation']['ros__parameters']
    centers = params['obstacle_centers']
    safety = AquariumSafety(
        half_width=params['aquarium_half_width'],
        half_height=params['aquarium_half_height'],
        robot_radius=params['robot_radius'],
        clearance=params['safety_clearance'],
        obstacles=[
            CircularObstacle(centers[2 * i], centers[2 * i + 1], radius)
            for i, radius in enumerate(params['obstacle_radii'])
        ],
    )
    samples = params['path_samples']
    routes = {
        'figure-eight': figure_eight(0.0, -0.65, 1.15, 0.42, samples),
        'orbit': circle(1.0, 0.8, 0.65, samples),
        'dance circle': circle(0.0, -0.7, 0.38, samples, loops=2),
        'dance s-curve': s_curve(0.0, -0.7, 1.1, 0.28, samples),
        'dance figure-eight': figure_eight(0.0, -0.7, 0.55, 0.28, samples),
        'home': [PathPoint(*params['home'])],
        'patrol': [
            PathPoint(x, y)
            for x, y in zip(params['patrol_points'][::2],
                            params['patrol_points'][1::2])
        ],
    }

    for name, points in routes.items():
        safety.validate(points, name)


def test_rejects_wall_and_obstacle_points():
    safety = aquarium_safety()

    assert not safety.is_safe(PathPoint(1.9, 0.0))
    assert not safety.is_safe(PathPoint(1.0, 0.8))
    assert safety.is_safe(PathPoint(0.0, 0.0))
