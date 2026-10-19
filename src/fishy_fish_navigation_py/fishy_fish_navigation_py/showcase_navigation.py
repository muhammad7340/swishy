"""Modular presentation and autonomous navigation modes for Swishy."""

import math
import random
import time
from typing import Callable, List, Optional, Sequence

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .navigation_patterns import AquariumSafety
from .navigation_patterns import circle
from .navigation_patterns import CircularObstacle
from .navigation_patterns import figure_eight
from .navigation_patterns import PathPoint
from .navigation_patterns import s_curve


CompletionCallback = Callable[[bool], None]


class ShowcaseNavigation(Node):
    """Run a selected navigation strategy while retaining Nav2 safety."""

    VALID_MODES = ('showcase', 'explorer', 'patrol', 'dance', 'interactive')

    def __init__(self) -> None:
        super().__init__('showcase_navigation')
        self._declare_parameters()

        self.mode = str(self.get_parameter('mode').value).lower()
        if self.mode not in self.VALID_MODES:
            modes = ', '.join(self.VALID_MODES)
            raise ValueError(f'Unknown mode "{self.mode}". Choose one of: {modes}')

        self.loop = bool(self.get_parameter('loop').value)
        self.pause_duration = float(self.get_parameter('pause_duration').value)
        self.server_timeout = float(
            self.get_parameter('action_server_timeout').value
        )
        self.goal_retry_count = int(
            self.get_parameter('goal_retry_count').value
        )
        self.goal_retry_delay = float(
            self.get_parameter('goal_retry_delay').value
        )
        self.feedback_interval = float(
            self.get_parameter('feedback_interval').value
        )
        self.path_samples = int(self.get_parameter('path_samples').value)
        self.home = self._point_parameter('home')
        self.safety = self._create_safety_model()

        self.navigate_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self._active_goal_handle = None
        self._busy = False
        self._last_feedback_time = 0.0
        self._pause_timers = []
        self._random = random.Random(
            int(self.get_parameter('explorer_seed').value)
        )
        self._explorer_completed = 0

        display_qos = QoSProfile(depth=1)
        display_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        display_qos.reliability = ReliabilityPolicy.RELIABLE
        self.status_publisher = self.create_publisher(
            String, '/swishy/showcase/status', display_qos
        )
        self.path_publisher = self.create_publisher(
            Path, '/swishy/showcase/path', display_qos
        )

        if self.mode == 'interactive':
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter('interactive_goal_topic').value),
                self._interactive_goal_callback,
                10,
            )
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter('detected_object_goal_topic').value),
                self._detected_object_goal_callback,
                10,
            )

        self._start_timer = self.create_timer(1.0, self._start_once)
        self._publish_status(f'Ready: {self.mode} mode')

    def _declare_parameters(self) -> None:
        self.declare_parameter('mode', 'showcase')
        self.declare_parameter('loop', False)
        self.declare_parameter('pause_duration', 1.5)
        self.declare_parameter('action_server_timeout', 20.0)
        self.declare_parameter('goal_retry_count', 2)
        self.declare_parameter('goal_retry_delay', 1.0)
        self.declare_parameter('feedback_interval', 1.0)
        self.declare_parameter('path_samples', 140)
        self.declare_parameter('home', [0.0, 0.0, 0.0])
        self.declare_parameter('aquarium_half_width', 2.0)
        self.declare_parameter('aquarium_half_height', 2.0)
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('safety_clearance', 0.12)
        self.declare_parameter(
            'obstacle_centers', [1.0, 0.8, -1.3, 0.85]
        )
        self.declare_parameter('obstacle_radii', [0.2, 0.22])
        self.declare_parameter(
            'patrol_points',
            [1.45, -1.45, 1.45, 1.45, -1.45, 1.45, -1.45, -1.45],
        )
        self.declare_parameter('explorer_seed', 42)
        self.declare_parameter('explorer_goal_count', 0)
        self.declare_parameter('interactive_goal_topic', '/goal_pose')
        self.declare_parameter(
            'detected_object_goal_topic', '/detected_object_goal'
        )

    def _point_parameter(self, name: str) -> PathPoint:
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f'{name} must contain [x, y, yaw]')
        return PathPoint(float(values[0]), float(values[1]), float(values[2]))

    def _create_safety_model(self) -> AquariumSafety:
        centers = list(self.get_parameter('obstacle_centers').value)
        radii = list(self.get_parameter('obstacle_radii').value)
        if len(centers) != 2 * len(radii):
            raise ValueError(
                'obstacle_centers must contain one x/y pair per obstacle radius'
            )

        obstacles = [
            CircularObstacle(
                float(centers[index * 2]),
                float(centers[index * 2 + 1]),
                float(radius),
            )
            for index, radius in enumerate(radii)
        ]
        return AquariumSafety(
            half_width=float(self.get_parameter('aquarium_half_width').value),
            half_height=float(self.get_parameter('aquarium_half_height').value),
            robot_radius=float(self.get_parameter('robot_radius').value),
            clearance=float(self.get_parameter('safety_clearance').value),
            obstacles=obstacles,
        )

    def _start_once(self) -> None:
        self._start_timer.cancel()
        starters = {
            'showcase': self._start_showcase,
            'explorer': self._start_explorer,
            'patrol': self._start_patrol,
            'dance': self._start_dance,
            'interactive': self._start_interactive,
        }
        starters[self.mode]()

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self.status_publisher.publish(message)
        self.get_logger().info(text)

    def _pose(self, point: PathPoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.z = math.sin(point.yaw / 2.0)
        pose.pose.orientation.w = math.cos(point.yaw / 2.0)
        return pose

    def _path_message(self, points: Sequence[PathPoint]) -> Path:
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self._pose(point) for point in points]
        return path

    def _wait_for_server(self, client: ActionClient, name: str) -> bool:
        self._publish_status(f'Waiting for Nav2 {name}...')
        if client.wait_for_server(timeout_sec=self.server_timeout):
            return True
        self.get_logger().error(
            f'Nav2 {name} is unavailable. Confirm all managed nodes are active.'
        )
        return False

    def _navigate_to(
        self,
        point: PathPoint,
        label: str,
        callback: CompletionCallback,
    ) -> None:
        if not self._wait_for_server(self.navigate_client, 'navigate_to_pose'):
            callback(False)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._pose(point)
        self._busy = True
        self._publish_status(
            f'{label}: navigating to ({point.x:.2f}, {point.y:.2f})'
        )
        future = self.navigate_client.send_goal_async(
            goal, feedback_callback=self._navigation_feedback
        )
        future.add_done_callback(
            lambda response: self._goal_response(response, callback, label)
        )

    def _follow_path(
        self,
        points: Sequence[PathPoint],
        label: str,
        callback: CompletionCallback,
    ) -> None:
        try:
            self.safety.validate(points, label)
        except ValueError as error:
            self.get_logger().error(str(error))
            callback(False)
            return

        if not self._wait_for_server(self.follow_path_client, 'follow_path'):
            callback(False)
            return

        path = self._path_message(points)
        self.path_publisher.publish(path)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        goal.progress_checker_id = 'progress_checker'
        self._busy = True
        self._publish_status(f'{label}: following {len(points)} smooth poses')
        future = self.follow_path_client.send_goal_async(
            goal, feedback_callback=self._path_feedback
        )
        future.add_done_callback(
            lambda response: self._goal_response(response, callback, label)
        )

    def _goal_response(
        self,
        future,
        callback: CompletionCallback,
        label: str,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy futures surface transport errors here
            self._busy = False
            self.get_logger().error(f'{label}: goal request failed: {error}')
            callback(False)
            return

        if not goal_handle.accepted:
            self._busy = False
            self.get_logger().error(f'{label}: Nav2 rejected the goal')
            callback(False)
            return

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._goal_result(result, callback, label)
        )

    def _goal_result(
        self,
        future,
        callback: CompletionCallback,
        label: str,
    ) -> None:
        self._active_goal_handle = None
        self._busy = False
        try:
            wrapped_result = future.result()
            succeeded = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            error_message = getattr(wrapped_result.result, 'error_msg', '')
        except Exception as error:
            self.get_logger().error(f'{label}: result failed: {error}')
            callback(False)
            return

        if succeeded:
            self._publish_status(f'{label}: complete')
        else:
            detail = f' ({error_message})' if error_message else ''
            self.get_logger().error(f'{label}: failed{detail}')
        callback(succeeded)

    def _feedback_due(self) -> bool:
        now = time.monotonic()
        if now - self._last_feedback_time < self.feedback_interval:
            return False
        self._last_feedback_time = now
        return True

    def _navigation_feedback(self, message) -> None:
        if not self._feedback_due():
            return
        distance = message.feedback.distance_remaining
        self.get_logger().info(f'Navigation remaining: {distance:.2f} m')

    def _path_feedback(self, message) -> None:
        if not self._feedback_due():
            return
        feedback = message.feedback
        self.get_logger().info(
            f'Path remaining: {feedback.distance_to_goal:.2f} m, '
            f'speed: {feedback.speed:.2f} m/s'
        )

    def _pause(
        self,
        label: str,
        callback: CompletionCallback,
        duration: Optional[float] = None,
    ) -> None:
        pause_duration = self.pause_duration if duration is None else duration
        self._publish_status(f'{label}: pausing for {pause_duration:.1f} s')
        timer = None

        def finish_pause() -> None:
            timer.cancel()
            if timer in self._pause_timers:
                self._pause_timers.remove(timer)
            callback(True)

        timer = self.create_timer(pause_duration, finish_pause)
        self._pause_timers.append(timer)

    def _run_steps(
        self,
        steps: Sequence[tuple],
        index: int,
        completion: CompletionCallback,
        attempt: int = 0,
    ) -> None:
        if index >= len(steps):
            completion(True)
            return

        kind, payload, label = steps[index]

        def next_step(succeeded: bool) -> None:
            if succeeded:
                self._run_steps(steps, index + 1, completion)
                return

            retryable = kind in ('navigate', 'path')
            if retryable and attempt < self.goal_retry_count:
                retry_number = attempt + 1
                self._publish_status(
                    f'{label}: transient failure; retry '
                    f'{retry_number}/{self.goal_retry_count}'
                )
                self._pause(
                    'Waiting before retry',
                    lambda _: self._run_steps(
                        steps, index, completion, retry_number
                    ),
                    duration=self.goal_retry_delay,
                )
                return

            completion(False)

        if kind == 'navigate':
            self._navigate_to(payload, label, next_step)
        elif kind == 'path':
            self._follow_path(payload, label, next_step)
        elif kind == 'pause':
            self._pause(label, next_step)
        else:
            self.get_logger().error(f'Unsupported sequence step: {kind}')
            completion(False)

    def _repeat_or_finish(self, succeeded: bool, restart: Callable) -> None:
        if not succeeded:
            self._publish_status(f'{self.mode.title()} mode stopped after a failure')
        elif self.loop:
            self._publish_status(f'{self.mode.title()} loop restarting')
            self._pause('Loop transition', lambda _: restart())
        else:
            self._publish_status(f'{self.mode.title()} mode finished')

    def _start_showcase(self) -> None:
        figure = figure_eight(0.0, -0.65, 1.15, 0.42, self.path_samples)
        orbit = circle(1.0, 0.8, 0.65, self.path_samples)
        steps = [
            ('navigate', figure[0], 'Entering the figure-eight'),
            ('path', figure, 'Figure-eight showcase'),
            ('pause', None, 'Looking around'),
            ('navigate', orbit[0], 'Approaching the center obstacle'),
            ('path', orbit, 'Obstacle orbit'),
            ('pause', None, 'Presentation pose'),
            ('navigate', self.home, 'Returning home'),
        ]
        self._run_steps(
            steps,
            0,
            lambda success: self._repeat_or_finish(
                success, self._start_showcase
            ),
        )

    def _start_dance(self) -> None:
        dance_circle = circle(
            0.0, -0.7, 0.38, self.path_samples, loops=2
        )
        dance_s = s_curve(0.0, -0.7, 1.1, 0.28, self.path_samples)
        dance_eight = figure_eight(
            0.0, -0.7, 0.55, 0.28, self.path_samples
        )
        steps = [
            ('navigate', dance_circle[0], 'Taking the dance floor'),
            ('path', dance_circle, 'Double circle'),
            ('navigate', dance_s[0], 'Changing dance pattern'),
            ('path', dance_s, 'S-curve dance'),
            ('navigate', dance_eight[0], 'Final dance pattern'),
            ('path', dance_eight, 'Compact figure-eight'),
            ('pause', None, 'Final pose'),
            ('navigate', self.home, 'Returning home'),
        ]
        self._run_steps(
            steps,
            0,
            lambda success: self._repeat_or_finish(success, self._start_dance),
        )

    def _patrol_points(self) -> List[PathPoint]:
        values = list(self.get_parameter('patrol_points').value)
        if len(values) < 4 or len(values) % 2:
            raise ValueError('patrol_points must contain at least two x/y pairs')

        points = []
        for index in range(0, len(values), 2):
            current_x = float(values[index])
            current_y = float(values[index + 1])
            next_index = (index + 2) % len(values)
            next_x = float(values[next_index])
            next_y = float(values[next_index + 1])
            points.append(
                PathPoint(
                    current_x,
                    current_y,
                    math.atan2(next_y - current_y, next_x - current_x),
                )
            )
        return points

    def _start_patrol(self) -> None:
        steps = []
        for index, point in enumerate(self._patrol_points(), start=1):
            steps.append(('navigate', point, f'Patrol station {index}'))
            steps.append(('pause', None, f'Inspecting station {index}'))
        steps.append(('navigate', self.home, 'Returning home'))
        self._run_steps(
            steps,
            0,
            lambda success: self._repeat_or_finish(success, self._start_patrol),
        )

    def _random_explorer_goal(self) -> PathPoint:
        for _ in range(200):
            point = PathPoint(
                self._random.uniform(-self.safety.x_limit, self.safety.x_limit),
                self._random.uniform(-self.safety.y_limit, self.safety.y_limit),
                self._random.uniform(-math.pi, math.pi),
            )
            if self.safety.is_safe(point):
                return point
        raise RuntimeError('Unable to sample a safe explorer goal')

    def _start_explorer(self) -> None:
        goal_limit = int(self.get_parameter('explorer_goal_count').value)
        if goal_limit > 0 and self._explorer_completed >= goal_limit:
            self._navigate_to(
                self.home,
                'Explorer returning home',
                lambda success: self._repeat_or_finish(success, self._reset_explorer),
            )
            return

        point = self._random_explorer_goal()
        goal_number = self._explorer_completed + 1
        self._navigate_to(
            point,
            f'Explorer goal {goal_number}',
            self._explorer_goal_complete,
        )

    def _explorer_goal_complete(self, succeeded: bool) -> None:
        if succeeded:
            self._explorer_completed += 1
        else:
            self.get_logger().warning(
                'Explorer goal was unreachable; selecting another safe goal'
            )
        self._pause('Scanning the aquarium', lambda _: self._start_explorer())

    def _reset_explorer(self) -> None:
        self._explorer_completed = 0
        self._start_explorer()

    def _start_interactive(self) -> None:
        goal_topic = str(self.get_parameter('interactive_goal_topic').value)
        object_topic = str(
            self.get_parameter('detected_object_goal_topic').value
        )
        self._publish_status(
            f'Interactive mode: waiting on {goal_topic} or {object_topic}'
        )

    def _interactive_goal_callback(self, pose: PoseStamped) -> None:
        self._accept_external_goal(pose, 'Interactive goal')

    def _detected_object_goal_callback(self, pose: PoseStamped) -> None:
        self._accept_external_goal(pose, 'Detected-object response')

    def _accept_external_goal(self, pose: PoseStamped, label: str) -> None:
        if self._busy:
            self.get_logger().warning(f'{label} ignored while another goal is active')
            return
        point = PathPoint(
            pose.pose.position.x,
            pose.pose.position.y,
            2.0 * math.atan2(
                pose.pose.orientation.z, pose.pose.orientation.w
            ),
        )
        if not self.safety.is_safe(point):
            self.get_logger().error(
                f'{label} ({point.x:.2f}, {point.y:.2f}) is outside the '
                'configured safety envelope'
            )
            return
        self._navigate_to(
            point,
            label,
            lambda success: self._publish_status(
                f'{label}: {"complete" if success else "failed"}; waiting'
            ),
        )

    def cancel_active_goal(self) -> None:
        if not rclpy.ok():
            return
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
        self._publish_status('Showcase navigation stopped')


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[ShowcaseNavigation] = None
    try:
        node = ShowcaseNavigation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f'showcase_navigation: {error}')
    finally:
        if node is not None:
            node.cancel_active_goal()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
