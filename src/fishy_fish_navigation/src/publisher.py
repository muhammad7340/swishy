#!/usr/bin/env python3

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class FishGaitController(Node):
    """Animate a bounded body wave that follows the commanded planar motion."""

    def __init__(self):
        super().__init__('fish_gait_controller')

        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('max_tail_angle', 0.55)
        self.declare_parameter('max_steering_bias', 0.28)
        self.declare_parameter('min_frequency', 1.2)
        self.declare_parameter('max_frequency', 2.4)
        self.declare_parameter('min_tail_fraction', 0.6)
        self.declare_parameter('max_joint_speed', 10.0)
        self.declare_parameter('command_timeout', 0.5)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_tail_angle = self.get_parameter('max_tail_angle').value
        self.max_steering_bias = self.get_parameter('max_steering_bias').value
        self.min_frequency = self.get_parameter('min_frequency').value
        self.max_frequency = self.get_parameter('max_frequency').value
        self.min_tail_fraction = self.get_parameter('min_tail_fraction').value
        self.max_joint_speed = self.get_parameter('max_joint_speed').value
        self.command_timeout = self.get_parameter('command_timeout').value

        self.linear_command = 0.0
        self.angular_command = 0.0
        self.activity = 0.0
        self.phase = 0.0
        self.positions = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.last_command_time = self.get_clock().now()
        self.last_update_time = self.last_command_time

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/fish_joint_controller/commands',
            10,
        )
        self.planar_publishers = [
            self.create_publisher(Float64, topic, 10)
            for topic in (
                '/planar_x_velocity',
                '/planar_y_velocity',
                '/planar_yaw_velocity',
            )
        ]
        self.create_timer(0.02, self.publish_joint_positions)

    def cmd_vel_callback(self, msg):
        self.linear_command = clamp(
            msg.linear.x / self.max_linear_speed,
            -1.0,
            1.0,
        )
        self.angular_command = clamp(
            msg.angular.z / self.max_angular_speed,
            -1.0,
            1.0,
        )
        self.last_command_time = self.get_clock().now()

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        self.yaw = math.atan2(sin_yaw, cos_yaw)

    def publish_planar_velocity(self, linear, angular):
        speed = linear * self.max_linear_speed
        velocities = (
            speed * math.cos(self.yaw),
            speed * math.sin(self.yaw),
            angular * self.max_angular_speed,
        )
        for publisher, velocity in zip(self.planar_publishers, velocities):
            message = Float64()
            message.data = velocity
            publisher.publish(message)

    def publish_joint_positions(self):
        now = self.get_clock().now()
        dt = clamp((now - self.last_update_time).nanoseconds * 1e-9, 0.0, 0.1)
        self.last_update_time = now

        command_age = (now - self.last_command_time).nanoseconds * 1e-9
        if command_age > self.command_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.linear_command
            angular = self.angular_command

        self.publish_planar_velocity(linear, angular)

        target_activity = max(abs(linear), 0.55 * abs(angular))
        activity_step = 3.0 * dt
        self.activity += clamp(
            target_activity - self.activity,
            -activity_step,
            activity_step,
        )

        frequency = self.min_frequency + (
            self.max_frequency - self.min_frequency
        ) * self.activity
        wave_direction = -1.0 if linear < -0.05 else 1.0
        self.phase = math.fmod(
            self.phase + wave_direction * 2.0 * math.pi * frequency * dt,
            2.0 * math.pi,
        )

        # Like a real fish, the tail keeps a near-constant swing while it swims
        # and beats faster to go faster: even slow swimming gets at least
        # min_tail_fraction of the full swing. The swing fades in over the
        # first 15% of activity so it still settles to rest when stopped.
        swimming = min(1.0, self.activity / 0.15)
        amplitude = self.max_tail_angle * swimming * (
            self.min_tail_fraction
            + (1.0 - self.min_tail_fraction) * self.activity
        )
        steering = self.max_steering_bias * angular
        phase_lag = math.pi / 3.0
        targets = [
            0.35 * steering + 0.45 * amplitude * math.sin(self.phase),
            0.65 * steering + 0.72 * amplitude * math.sin(self.phase - phase_lag),
            steering + amplitude * math.sin(self.phase - 2.0 * phase_lag),
        ]

        max_position_step = self.max_joint_speed * dt
        self.positions = [
            clamp(
                current + clamp(
                    target - current,
                    -max_position_step,
                    max_position_step,
                ),
                -0.75,
                0.75,
            )
            for current, target in zip(self.positions, targets)
        ]

        message = Float64MultiArray()
        message.data = self.positions
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    controller = FishGaitController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
