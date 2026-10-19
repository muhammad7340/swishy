"""Geometry and reusable motion patterns for Swishy's presentation modes."""

from dataclasses import dataclass
import math
from typing import Callable, Iterable, List, Sequence


@dataclass(frozen=True)
class PathPoint:
    """A planar path point with a tangent-aligned heading."""

    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class CircularObstacle:
    """A conservative circular approximation of an aquarium obstacle."""

    x: float
    y: float
    radius: float


class AquariumSafety:
    """Validate presentation paths against walls and known static obstacles."""

    def __init__(
        self,
        half_width: float,
        half_height: float,
        robot_radius: float,
        clearance: float,
        obstacles: Sequence[CircularObstacle],
    ) -> None:
        self.half_width = half_width
        self.half_height = half_height
        self.robot_radius = robot_radius
        self.clearance = clearance
        self.obstacles = obstacles

    @property
    def x_limit(self) -> float:
        return self.half_width - self.robot_radius - self.clearance

    @property
    def y_limit(self) -> float:
        return self.half_height - self.robot_radius - self.clearance

    def is_safe(self, point: PathPoint) -> bool:
        if abs(point.x) > self.x_limit or abs(point.y) > self.y_limit:
            return False

        for obstacle in self.obstacles:
            required_distance = (
                obstacle.radius + self.robot_radius + self.clearance
            )
            if math.hypot(point.x - obstacle.x, point.y - obstacle.y) < (
                required_distance
            ):
                return False
        return True

    def validate(self, points: Iterable[PathPoint], name: str) -> None:
        for index, point in enumerate(points):
            if not self.is_safe(point):
                raise ValueError(
                    f'{name} point {index} ({point.x:.2f}, {point.y:.2f}) '
                    'violates the configured aquarium safety envelope'
                )


def _with_tangent_headings(points: Sequence[PathPoint]) -> List[PathPoint]:
    if len(points) < 2:
        return list(points)

    result = []
    for index, point in enumerate(points):
        previous = points[max(0, index - 1)]
        following = points[min(len(points) - 1, index + 1)]
        yaw = math.atan2(following.y - previous.y, following.x - previous.x)
        result.append(PathPoint(point.x, point.y, yaw))
    return result


def _sample(
    curve: Callable[[float], tuple[float, float]],
    samples: int,
    loops: int = 1,
) -> List[PathPoint]:
    count = max(8, samples) * max(1, loops)
    points = []
    for index in range(count + 1):
        phase = 2.0 * math.pi * loops * index / count
        x, y = curve(phase)
        points.append(PathPoint(x, y))
    return _with_tangent_headings(points)


def figure_eight(
    center_x: float,
    center_y: float,
    radius_x: float,
    radius_y: float,
    samples: int,
    loops: int = 1,
) -> List[PathPoint]:
    """Create a smooth Gerono figure-eight path."""
    return _sample(
        lambda phase: (
            center_x + radius_x * math.sin(phase),
            center_y + radius_y * math.sin(2.0 * phase),
        ),
        samples,
        loops,
    )


def circle(
    center_x: float,
    center_y: float,
    radius: float,
    samples: int,
    loops: int = 1,
    start_angle: float = math.pi,
) -> List[PathPoint]:
    """Create a counter-clockwise circular path."""
    return _sample(
        lambda phase: (
            center_x + radius * math.cos(phase + start_angle),
            center_y + radius * math.sin(phase + start_angle),
        ),
        samples,
        loops,
    )


def s_curve(
    center_x: float,
    center_y: float,
    width: float,
    height: float,
    samples: int,
) -> List[PathPoint]:
    """Create a closed, flowing S-shaped dance path."""
    outbound = []
    count = max(12, samples // 2)
    for index in range(count + 1):
        progress = index / count
        x = center_x - width / 2.0 + width * progress
        y = center_y + height * math.sin(2.0 * math.pi * progress)
        outbound.append(PathPoint(x, y))

    inbound = []
    for index in range(1, count + 1):
        progress = index / count
        x = center_x + width / 2.0 - width * progress
        y = center_y - height * math.sin(2.0 * math.pi * progress)
        inbound.append(PathPoint(x, y))

    return _with_tangent_headings(outbound + inbound)
