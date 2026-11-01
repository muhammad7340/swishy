#!/usr/bin/env python3

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


# Tail segments behind joint_1, joint_2 and joint_3 (m), joint_3 to fin tip
# measured from the STL meshes.
TAIL_SEGMENTS = (0.05, 0.0495, 0.102)
WAVE_SHAPE = (0.45, 0.72, 1.0)
STEERING_SHAPE = (0.35, 0.65, 1.0)
PHASE_LAG = math.pi / 3.0


def tail_angles(phase, amplitude, steering):
    """Joint angles of the travelling body wave (the gait itself)."""
    return [
        bias * steering + shape * amplitude * math.sin(phase - i * PHASE_LAG)
        for i, (shape, bias) in enumerate(zip(WAVE_SHAPE, STEERING_SHAPE))
    ]


def tail_tip(angles):
    """Sideways offset h and slope dh/dx of the fin tip in the body frame."""
    heading = 0.0
    offset = 0.0
    for angle, length in zip(angles, TAIL_SEGMENTS):
        heading += angle
        offset += length * math.sin(heading)
    return offset, math.tan(heading)


def lighthill_thrust(added_mass, tip_velocity, tip_slope, speed):
    """Lighthill's elongated-body thrust T = 1/2 m (h_t^2 - U^2 h_x^2).

    added_mass is the fluid mass per metre carried by the fin, 1/4 pi rho s^2.
    Thrust grows with how fast the tip sweeps sideways and falls as the body
    speeds up, vanishing once the fish swims as fast as its own body wave.
    """
    return 0.5 * added_mass * (
        tip_velocity * tip_velocity - speed * speed * tip_slope * tip_slope
    )


def mean_gait_thrust(added_mass, frequency, amplitude, speed, samples=200):
    """Thrust averaged over one tail beat of the steady gait."""
    period = 1.0 / frequency
    dt = period / samples
    total = 0.0
    previous, _ = tail_tip(tail_angles(0.0, amplitude, 0.0))
    for step in range(1, samples + 1):
        phase = 2.0 * math.pi * frequency * step * dt
        offset, slope = tail_tip(tail_angles(phase, amplitude, 0.0))
        total += lighthill_thrust(
            added_mass, (offset - previous) / dt, slope, speed)
        previous = offset
    return total / samples


def tail_sweep(amplitude):
    """Peak-to-peak sideways travel of the fin tip over one beat (m)."""
    offsets = [
        tail_tip(tail_angles(2.0 * math.pi * k / 24.0, amplitude, 0.0))[0]
        for k in range(24)
    ]
    return max(offsets) - min(offsets)


class FishGaitController(Node):
    """Swim with a travelling body wave whose thrust sets the forward speed.

    /cmd_vel sets how hard the tail beats; the forward speed then comes from
    a one-dimensional force balance, (m + m_a) dU/dt = T - D, with Lighthill
    thrust T from the fin tip's motion and quadratic water drag D. That speed
    drives the planar joints, so the fish accelerates, coasts and brakes, and
    a tail that stops beating stops pushing. Turning stays kinematic.
    """

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
        # Swim physics. hybrid_swimming false restores the old behaviour,
        # where /cmd_vel moves the body directly.
        self.declare_parameter('hybrid_swimming', True)
        self.declare_parameter('water_density', 1000.0)
        self.declare_parameter('fin_span', 0.13)
        self.declare_parameter('body_mass', 0.43)
        self.declare_parameter('surge_added_mass', 0.05)
        self.declare_parameter('linear_drag', 0.3)
        self.declare_parameter('top_speed_margin', 1.2)
        self.declare_parameter('brake_drag_factor', 4.0)
        self.declare_parameter('hover_drag_factor', 30.0)
        self.declare_parameter('speed_gain', 2.0)
        self.declare_parameter('speed_integral_gain', 1.5)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_tail_angle = self.get_parameter('max_tail_angle').value
        self.max_steering_bias = self.get_parameter('max_steering_bias').value
        self.min_frequency = self.get_parameter('min_frequency').value
        self.max_frequency = self.get_parameter('max_frequency').value
        self.min_tail_fraction = self.get_parameter('min_tail_fraction').value
        self.max_joint_speed = self.get_parameter('max_joint_speed').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.hybrid_swimming = self.get_parameter('hybrid_swimming').value
        self.linear_drag = self.get_parameter('linear_drag').value
        self.brake_drag_factor = self.get_parameter('brake_drag_factor').value
        self.hover_drag_factor = self.get_parameter('hover_drag_factor').value
        self.speed_gain = self.get_parameter('speed_gain').value
        self.speed_integral_gain = self.get_parameter(
            'speed_integral_gain').value
        self.fin_added_mass = 0.25 * math.pi * self.get_parameter(
            'water_density').value * self.get_parameter('fin_span').value ** 2
        self.surge_mass = self.get_parameter('body_mass').value + self.get_parameter(
            'surge_added_mass').value

        # Size the drag so the full-power gait balances it slightly above
        # max_linear_speed, leaving the speed loop headroom to reach it.
        self.top_speed = self.max_linear_speed * self.get_parameter(
            'top_speed_margin').value
        full_thrust = mean_gait_thrust(
            self.fin_added_mass,
            self.max_frequency,
            self.max_tail_angle,
            self.top_speed,
        )
        self.quadratic_drag = max(
            0.0,
            (full_thrust - self.linear_drag * self.top_speed)
            / self.top_speed ** 2,
        )
        if self.hybrid_swimming:
            self.get_logger().info(
                f'Hybrid swimming: fin added mass {self.fin_added_mass:.2f} kg/m, '
                f'full-gait thrust {full_thrust:.2f} N at {self.top_speed:.2f} m/s, '
                f'drag {self.quadratic_drag:.2f} U|U| + {self.linear_drag:.2f} U N'
            )

        self.linear_command = 0.0
        self.angular_command = 0.0
        self.activity = 0.0
        self.phase = 0.0
        self.positions = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.speed = 0.0
        self.thrust = 0.0
        self.speed_integral = 0.0
        self.tip_offset = 0.0
        self.brake = 1.0
        self.hovering = True
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
        self.swim_publishers = {
            name: self.create_publisher(Float64, f'/swim/{name}', 10)
            for name in ('speed', 'thrust', 'drag', 'strouhal')
        }
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

    def publish_planar_velocity(self, speed, angular):
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

        if self.hybrid_swimming:
            # Beat harder when slower than commanded: feed-forward from the
            # commanded share of top speed plus a PI correction.
            target_speed = linear * self.max_linear_speed
            error = (abs(target_speed) - abs(self.speed)) / self.max_linear_speed
            if abs(target_speed) < 1e-3:
                self.speed_integral = 0.0
                effort = 0.0
            else:
                self.speed_integral = clamp(
                    self.speed_integral + self.speed_integral_gain * error * dt,
                    -0.5,
                    0.5,
                )
                effort = clamp(
                    abs(target_speed) / self.top_speed
                    + self.speed_gain * error
                    + self.speed_integral,
                    0.0,
                    1.0,
                )
            target_activity = max(effort, 0.55 * abs(angular))
            # When the tail beats harder to turn than the speed needs, only
            # the swimming share of the beat pushes forward; the rest turns.
            propulsive_share = effort / target_activity if target_activity > 1e-3 else 0.0
        else:
            self.publish_planar_velocity(
                linear * self.max_linear_speed, angular)
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
        targets = tail_angles(self.phase, amplitude, steering)

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

        if self.hybrid_swimming:
            self.swim(linear, angular, frequency, amplitude, wave_direction,
                      propulsive_share, dt)

    def swim(self, linear, angular, frequency, amplitude, wave_direction,
             propulsive_share, dt):
        """Advance the forward speed by one step of thrust against drag."""
        offset, slope = tail_tip(self.positions)
        tip_velocity = (offset - self.tip_offset) / dt if dt > 0.0 else 0.0
        self.tip_offset = offset
        if dt <= 0.0:
            return

        # Real fish brake by flaring their fins: extra drag that grows with
        # how far the fish is above the commanded speed, so easing off glides.
        # Asked to stop, it brakes hard and holds station (hovers): Nav2
        # treats zero as stop and turns on the spot at the goal, so a long
        # glide would carry the fish out of the goal area. Fins take a
        # moment to flare, hence the lag.
        target_speed = linear * self.max_linear_speed
        excess = abs(self.speed) if target_speed * self.speed < 0.0 else (
            abs(self.speed) - abs(target_speed))
        self.hovering = abs(target_speed) < 0.02
        if self.hovering:
            brake_target = self.hover_drag_factor
        else:
            brake_target = 1.0 + (self.brake_drag_factor - 1.0) * clamp(
                excess / 0.1, 0.0, 1.0)
        self.brake += (brake_target - self.brake) * clamp(dt / 0.15, 0.0, 1.0)

        # A forward wave pushes forward, a reversed wave backs the fish up.
        # While hovering (turning on the spot) the pectoral fins back-paddle
        # against the tail's push, so the turn does not drift forward.
        thrust = 0.0 if self.hovering else wave_direction * propulsive_share * max(
            0.0,
            lighthill_thrust(self.fin_added_mass, tip_velocity, slope, self.speed),
        )
        # Smooth the twice-per-beat thrust pulses over about half a beat.
        self.thrust += (thrust - self.thrust) * clamp(dt / 0.2, 0.0, 1.0)

        quadratic = self.quadratic_drag * self.brake
        # Drag is stiff (water stops this light fish within a few steps), so
        # solve the step implicitly: m (U' - U) = (T - k U'|U'| - c U') dt,
        # a quadratic in U' with one root of the right sign.
        momentum = self.surge_mass * self.speed + self.thrust * dt
        direction = 1.0 if momentum >= 0.0 else -1.0
        a = quadratic * dt
        b = self.surge_mass + self.linear_drag * dt
        c = abs(momentum)
        root = c / b if a < 1e-9 else (
            -b + math.sqrt(b * b + 4.0 * a * c)) / (2.0 * a)
        self.speed = direction * root
        drag = quadratic * self.speed * abs(self.speed) + self.linear_drag * self.speed
        if abs(self.speed) < 1e-3 and abs(self.thrust) < 1e-3:
            self.speed = 0.0
        self.publish_planar_velocity(self.speed, angular)

        sweep = tail_sweep(amplitude)
        strouhal = frequency * sweep / abs(self.speed) if abs(self.speed) > 0.05 else 0.0
        for name, value in (
            ('speed', self.speed),
            ('thrust', self.thrust),
            ('drag', drag),
            ('strouhal', strouhal),
        ):
            message = Float64()
            message.data = value
            self.swim_publishers[name].publish(message)


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
