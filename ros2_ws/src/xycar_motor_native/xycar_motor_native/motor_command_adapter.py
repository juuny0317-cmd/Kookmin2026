"""Convert competition motor commands into safe VESC commands."""

import math
from typing import Tuple

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64
from vesc_msgs.msg import VescStateStamped


def clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(value, minimum), maximum)


def convert_command(
    angle: float,
    speed: float,
    angle_min: float = -42.0,
    angle_max: float = 42.0,
    speed_min: float = -27.0,
    speed_max: float = 27.0,
    steering_scale: float = -0.0068,
    speed_scale: float = 0.08,
) -> Tuple[float, float]:
    """Return steering radians and m/s using the calibrated ASUS values."""
    if not math.isfinite(angle) or not math.isfinite(speed):
        raise ValueError('motor command values must be finite')
    steering = clamp(angle, angle_min, angle_max) * steering_scale
    velocity = clamp(speed, speed_min, speed_max) * speed_scale
    return steering, velocity


def convert_duty_command(
    angle: float,
    speed: float,
    angle_min: float = -42.0,
    angle_max: float = 42.0,
    speed_min: float = -27.0,
    speed_max: float = 27.0,
    steering_scale: float = -0.0068,
    duty_cycle_per_speed_unit: float = 0.006,
    steering_to_servo_gain: float = -1.2135,
    steering_to_servo_offset: float = 0.5004,
    servo_min: float = 0.15,
    servo_max: float = 0.85,
) -> Tuple[float, float]:
    """Return VESC servo position and duty cycle for an Xycar command."""
    if not math.isfinite(angle) or not math.isfinite(speed):
        raise ValueError('motor command values must be finite')
    clamped_angle = clamp(angle, angle_min, angle_max)
    steering = clamped_angle * steering_scale
    servo = clamp(
        steering_to_servo_gain * steering + steering_to_servo_offset,
        servo_min,
        servo_max,
    )
    # The legacy calibration lands just short of both configured servo limits
    # (0.1538 and 0.8470). At a saturated Xycar steering command, explicitly
    # use the same safe endpoints enforced by the original ROS 1 VESC driver.
    if clamped_angle <= angle_min:
        servo = servo_min
    elif clamped_angle >= angle_max:
        servo = servo_max
    duty_cycle = (
        clamp(speed, speed_min, speed_max) * duty_cycle_per_speed_unit)
    return servo, duty_cycle


def update_erpm_pi(
    target_erpm: float,
    measured_erpm: float,
    feedforward_duty: float,
    integral_duty: float,
    dt: float,
    kp: float,
    ki: float,
    integral_limit: float,
    maximum_duty: float,
) -> Tuple[float, float]:
    """Return a sign-safe duty command and updated PI integral."""
    if not all(math.isfinite(value) for value in (
            target_erpm, measured_erpm, feedforward_duty,
            integral_duty, dt, kp, ki, integral_limit, maximum_duty)):
        raise ValueError('ERPM controller values must be finite')
    if dt <= 0.0 or kp < 0.0 or ki < 0.0:
        raise ValueError('ERPM controller timing and gains must be positive')
    if integral_limit < 0.0 or maximum_duty <= 0.0:
        raise ValueError('ERPM controller limits must be positive')
    if target_erpm == 0.0:
        return 0.0, 0.0

    direction = 1.0 if target_erpm > 0.0 else -1.0
    target_magnitude = abs(target_erpm)
    measured_in_direction = direction * measured_erpm
    error = target_magnitude - measured_in_direction
    minimum_integral = -abs(feedforward_duty)
    next_integral = clamp(
        integral_duty + ki * error * dt,
        minimum_integral,
        integral_limit,
    )
    raw_magnitude = abs(feedforward_duty) + kp * error + next_integral
    duty_magnitude = clamp(raw_magnitude, 0.0, maximum_duty)
    return direction * duty_magnitude, next_integral


def slew_limit(current: float, target: float, rate_per_second: float,
               dt: float) -> float:
    """Move current toward target without exceeding the configured rate."""
    if rate_per_second <= 0.0 or dt <= 0.0:
        raise ValueError('slew rate and time step must be positive')
    maximum_step = rate_per_second * dt
    return current + clamp(target - current, -maximum_step, maximum_step)


def fixed_startup_duty(target_erpm: float, startup_duty: float) -> float:
    """Return the fixed, direction-aware duty used during breakaway."""
    if not math.isfinite(target_erpm) or not math.isfinite(startup_duty):
        raise ValueError('startup duty values must be finite')
    if not 0.0 <= startup_duty <= 1.0:
        raise ValueError('startup duty must be between 0 and 1')
    if target_erpm == 0.0:
        return 0.0
    return math.copysign(startup_duty, target_erpm)


def startup_boost_required(
    target_erpm: float,
    measured_erpm: float,
    timer_active: bool,
    exit_erpm: float,
    exit_target_ratio: float,
    exit_confirmation_elapsed_s: float = 0.0,
    exit_confirmation_s: float = 0.0,
) -> bool:
    """Return whether sensorless BLDC breakaway boost is still required."""
    if not all(math.isfinite(value) for value in (
            target_erpm, measured_erpm, exit_erpm, exit_target_ratio,
            exit_confirmation_elapsed_s, exit_confirmation_s)):
        raise ValueError('startup boost values must be finite')
    if exit_erpm <= 0.0 or not 0.0 < exit_target_ratio <= 1.0:
        raise ValueError('startup boost exit limits must be positive')
    if exit_confirmation_elapsed_s < 0.0 or exit_confirmation_s < 0.0:
        raise ValueError('startup confirmation times cannot be negative')
    if not timer_active or target_erpm == 0.0:
        return False
    direction = 1.0 if target_erpm > 0.0 else -1.0
    measured_in_direction = direction * measured_erpm
    exit_threshold = min(
        exit_erpm,
        abs(target_erpm) * exit_target_ratio,
    )
    if measured_in_direction < exit_threshold:
        return True
    return exit_confirmation_elapsed_s < exit_confirmation_s


class MotorCommandAdapter(Node):
    """Publish fixed-rate VESC commands with feedback and a watchdog."""

    def __init__(self) -> None:
        super().__init__('xycar_motor_adapter')
        self.declare_parameter('input_topic', '/xycar_motor')
        self.declare_parameter('output_topic', '/ackermann_cmd')
        self.declare_parameter('angle_min', -42.0)
        self.declare_parameter('angle_max', 42.0)
        self.declare_parameter('speed_min', -27.0)
        self.declare_parameter('speed_max', 27.0)
        self.declare_parameter('steering_scale_rad_per_unit', -0.0068)
        self.declare_parameter('speed_scale_mps_per_unit', 0.08)
        self.declare_parameter('control_mode', 'duty_speed')
        self.declare_parameter(
            'duty_cycle_output_topic', '/commands/motor/duty_cycle')
        self.declare_parameter(
            'servo_output_topic', '/commands/servo/position')
        self.declare_parameter('duty_cycle_per_speed_unit', 0.006)
        self.declare_parameter('startup_duty_cycle', 0.08)
        self.declare_parameter('startup_duration_s', 1.20)
        self.declare_parameter('startup_exit_erpm', 1600.0)
        self.declare_parameter('startup_exit_target_ratio', 0.95)
        self.declare_parameter('startup_exit_confirm_s', 0.05)
        self.declare_parameter('erpm_state_topic', '/sensors/core')
        self.declare_parameter('erpm_per_speed_unit', 369.12)
        self.declare_parameter('erpm_kp_duty_per_erpm', 0.000010)
        self.declare_parameter('erpm_ki_duty_per_erpm_s', 0.000010)
        self.declare_parameter('erpm_integral_limit_duty', 0.10)
        self.declare_parameter('erpm_startup_integral_duty', 0.026)
        self.declare_parameter('erpm_max_duty', 0.25)
        self.declare_parameter('erpm_filter_alpha', 0.25)
        self.declare_parameter('erpm_telemetry_timeout_s', 0.25)
        self.declare_parameter('duty_slew_rate_per_s', 0.30)
        self.declare_parameter('steering_to_servo_gain', -1.2135)
        self.declare_parameter('steering_to_servo_offset', 0.5004)
        self.declare_parameter('servo_min', 0.15)
        self.declare_parameter('servo_max', 0.85)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('watchdog_timeout_s', 0.30)

        self._angle_min = float(self.get_parameter('angle_min').value)
        self._angle_max = float(self.get_parameter('angle_max').value)
        self._speed_min = float(self.get_parameter('speed_min').value)
        self._speed_max = float(self.get_parameter('speed_max').value)
        self._steering_scale = float(
            self.get_parameter('steering_scale_rad_per_unit').value)
        self._speed_scale = float(
            self.get_parameter('speed_scale_mps_per_unit').value)
        self._control_mode = str(
            self.get_parameter('control_mode').value).strip().lower()
        self._duty_scale = float(
            self.get_parameter('duty_cycle_per_speed_unit').value)
        self._startup_duty = float(
            self.get_parameter('startup_duty_cycle').value)
        self._startup_duration = float(
            self.get_parameter('startup_duration_s').value)
        self._startup_exit_erpm = float(
            self.get_parameter('startup_exit_erpm').value)
        self._startup_exit_target_ratio = float(
            self.get_parameter('startup_exit_target_ratio').value)
        self._startup_exit_confirm = float(
            self.get_parameter('startup_exit_confirm_s').value)
        self._erpm_per_speed_unit = float(
            self.get_parameter('erpm_per_speed_unit').value)
        self._erpm_kp = float(
            self.get_parameter('erpm_kp_duty_per_erpm').value)
        self._erpm_ki = float(
            self.get_parameter('erpm_ki_duty_per_erpm_s').value)
        self._erpm_integral_limit = float(
            self.get_parameter('erpm_integral_limit_duty').value)
        self._erpm_startup_integral = float(
            self.get_parameter('erpm_startup_integral_duty').value)
        self._erpm_max_duty = float(
            self.get_parameter('erpm_max_duty').value)
        self._erpm_filter_alpha = float(
            self.get_parameter('erpm_filter_alpha').value)
        self._erpm_telemetry_timeout = float(
            self.get_parameter('erpm_telemetry_timeout_s').value)
        self._duty_slew_rate = float(
            self.get_parameter('duty_slew_rate_per_s').value)
        self._steering_to_servo_gain = float(
            self.get_parameter('steering_to_servo_gain').value)
        self._steering_to_servo_offset = float(
            self.get_parameter('steering_to_servo_offset').value)
        self._servo_min = float(self.get_parameter('servo_min').value)
        self._servo_max = float(self.get_parameter('servo_max').value)
        self._watchdog_timeout = float(
            self.get_parameter('watchdog_timeout_s').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        if self._angle_min > self._angle_max:
            raise ValueError('angle_min cannot exceed angle_max')
        if self._speed_min > self._speed_max:
            raise ValueError('speed_min cannot exceed speed_max')
        if self._servo_min > self._servo_max:
            raise ValueError('servo_min cannot exceed servo_max')
        if self._control_mode not in ('speed', 'duty_cycle', 'duty_speed'):
            raise ValueError(
                'control_mode must be speed, duty_cycle, or duty_speed')
        if self._duty_scale <= 0.0:
            raise ValueError('duty_cycle_per_speed_unit must be positive')
        if not 0.0 <= self._startup_duty <= 1.0:
            raise ValueError('startup_duty_cycle must be between 0 and 1')
        if self._startup_duration < 0.0:
            raise ValueError('startup_duration_s cannot be negative')
        if self._startup_exit_erpm <= 0.0:
            raise ValueError('startup_exit_erpm must be positive')
        if not 0.0 < self._startup_exit_target_ratio <= 1.0:
            raise ValueError(
                'startup_exit_target_ratio must be in (0, 1]')
        if self._startup_exit_confirm < 0.0:
            raise ValueError('startup_exit_confirm_s cannot be negative')
        if self._erpm_per_speed_unit <= 0.0:
            raise ValueError('erpm_per_speed_unit must be positive')
        if self._erpm_kp < 0.0 or self._erpm_ki < 0.0:
            raise ValueError('ERPM PI gains cannot be negative')
        if self._erpm_integral_limit < 0.0:
            raise ValueError('erpm_integral_limit_duty cannot be negative')
        if not 0.0 <= self._erpm_startup_integral <= self._erpm_integral_limit:
            raise ValueError(
                'erpm_startup_integral_duty must be within the integral limit')
        if not 0.0 < self._erpm_max_duty <= 1.0:
            raise ValueError('erpm_max_duty must be in (0, 1]')
        if not 0.0 < self._erpm_filter_alpha <= 1.0:
            raise ValueError('erpm_filter_alpha must be in (0, 1]')
        if self._erpm_telemetry_timeout <= 0.0:
            raise ValueError('erpm_telemetry_timeout_s must be positive')
        if self._duty_slew_rate <= 0.0:
            raise ValueError('duty_slew_rate_per_s must be positive')
        if publish_rate <= 0.0 or self._watchdog_timeout <= 0.0:
            raise ValueError(
                'publish rate and watchdog timeout must be positive')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._publisher = None
        self._duty_publisher = None
        self._servo_publisher = None
        if self._control_mode == 'speed':
            self._publisher = self.create_publisher(
                AckermannDriveStamped, output_topic, 10)
            output_description = output_topic
        else:
            duty_topic = str(
                self.get_parameter('duty_cycle_output_topic').value)
            servo_topic = str(self.get_parameter('servo_output_topic').value)
            self._duty_publisher = self.create_publisher(
                Float64, duty_topic, 10)
            self._servo_publisher = self.create_publisher(
                Float64, servo_topic, 10)
            output_description = f'{duty_topic} + {servo_topic}'
        self._subscription = self.create_subscription(
            Float32MultiArray, input_topic, self._command_callback, 10)
        self._state_subscription = None
        if self._control_mode == 'duty_speed':
            state_topic = str(self.get_parameter('erpm_state_topic').value)
            self._state_subscription = self.create_subscription(
                VescStateStamped, state_topic,
                self._state_callback, 20)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish)

        self._last_command_time = None
        self._steering = 0.0
        self._speed = 0.0
        self._servo = self._steering_to_servo_offset
        self._duty_cycle = 0.0
        self._target_erpm = 0.0
        self._filtered_erpm = None
        self._last_state_time = None
        self._last_control_time = self.get_clock().now()
        self._erpm_integral = 0.0
        self._controlled_duty = 0.0
        self._telemetry_warning_active = False
        self._motion_sign = 0
        self._startup_until = None
        self._startup_confirm_since = None
        self._watchdog_active = True
        self.get_logger().info(
            f'Safe motor adapter ({self._control_mode}): '
            f'{input_topic} -> {output_description}; '
            f'watchdog={self._watchdog_timeout:.2f}s')

    def _state_callback(self, message: VescStateStamped) -> None:
        measured_erpm = float(message.state.speed)
        if not math.isfinite(measured_erpm):
            self.get_logger().error(
                'Rejected non-finite VESC ERPM telemetry',
                throttle_duration_sec=2.0,
            )
            return
        if self._filtered_erpm is None:
            self._filtered_erpm = measured_erpm
        else:
            alpha = self._erpm_filter_alpha
            self._filtered_erpm = (
                alpha * measured_erpm
                + (1.0 - alpha) * self._filtered_erpm)
        self._last_state_time = self.get_clock().now()
        self._telemetry_warning_active = False

    def _command_callback(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            self.get_logger().error(
                'Rejected /xycar_motor message: expected [angle, speed]',
                throttle_duration_sec=2.0,
            )
            return
        try:
            angle_value = float(message.data[0])
            speed_value = float(message.data[1])
            steering, speed = convert_command(
                angle_value,
                speed_value,
                self._angle_min,
                self._angle_max,
                self._speed_min,
                self._speed_max,
                self._steering_scale,
                self._speed_scale,
            )
            servo, duty_cycle = convert_duty_command(
                angle_value,
                speed_value,
                self._angle_min,
                self._angle_max,
                self._speed_min,
                self._speed_max,
                self._steering_scale,
                self._duty_scale,
                self._steering_to_servo_gain,
                self._steering_to_servo_offset,
                self._servo_min,
                self._servo_max,
            )
        except ValueError as error:
            self.get_logger().error(
                f'Rejected unsafe motor command: {error}',
                throttle_duration_sec=2.0,
            )
            return

        self._steering = steering
        self._speed = speed
        self._servo = servo
        self._duty_cycle = duty_cycle
        self._target_erpm = (
            clamp(speed_value, self._speed_min, self._speed_max)
            * self._erpm_per_speed_unit)
        now = self.get_clock().now()
        new_sign = 1 if duty_cycle > 0.0 else -1 if duty_cycle < 0.0 else 0
        if new_sign == 0:
            self._motion_sign = 0
            self._startup_until = None
            self._startup_confirm_since = None
            self._erpm_integral = 0.0
            self._controlled_duty = 0.0
        elif new_sign != self._motion_sign:
            self._motion_sign = new_sign
            self._startup_until = now + Duration(
                seconds=self._startup_duration)
            self._startup_confirm_since = None
            # The recorded ground run required about 5% duty to sustain speed
            # 4. Seed the PI load estimate so the breakaway boost hands off to
            # usable torque instead of dropping back to the 2.4% feedforward.
            self._erpm_integral = self._erpm_startup_integral
            self._controlled_duty = 0.0
        self._last_command_time = now
        self._watchdog_active = False

    def _publish(self) -> None:
        now = self.get_clock().now()
        timed_out = self._last_command_time is None
        if self._last_command_time is not None:
            age = (now - self._last_command_time).nanoseconds / 1e9
            timed_out = age > self._watchdog_timeout

        steering = 0.0 if timed_out else self._steering
        speed = 0.0 if timed_out else self._speed
        servo = self._steering_to_servo_offset if timed_out else self._servo
        duty_cycle = 0.0 if timed_out else self._duty_cycle
        if timed_out and not self._watchdog_active:
            self.get_logger().warning(
                'Motor command watchdog expired; commanding zero speed')
            self._watchdog_active = True
            self._motion_sign = 0
            self._startup_until = None
            self._startup_confirm_since = None
            self._erpm_integral = 0.0
            self._controlled_duty = 0.0

        if self._control_mode == 'speed':
            output = AckermannDriveStamped()
            output.header.stamp = now.to_msg()
            output.header.frame_id = 'base_link'
            output.drive.steering_angle = float(steering)
            output.drive.speed = float(speed)
            self._publisher.publish(output)
            return

        if self._control_mode == 'duty_speed':
            control_dt = (now - self._last_control_time).nanoseconds / 1e9
            control_dt = clamp(control_dt, 0.01, 0.10)
            self._last_control_time = now
            state_timed_out = self._last_state_time is None
            if self._last_state_time is not None:
                state_age = (now - self._last_state_time).nanoseconds / 1e9
                state_timed_out = state_age > self._erpm_telemetry_timeout

            if timed_out or state_timed_out or self._target_erpm == 0.0:
                duty_cycle = 0.0
                self._erpm_integral = 0.0
                self._controlled_duty = 0.0
                if (state_timed_out and not timed_out
                        and not self._telemetry_warning_active):
                    self.get_logger().warning(
                        'VESC ERPM telemetry expired; commanding zero duty')
                    self._telemetry_warning_active = True
            else:
                target_duty, self._erpm_integral = update_erpm_pi(
                    self._target_erpm,
                    float(self._filtered_erpm),
                    self._duty_cycle,
                    self._erpm_integral,
                    control_dt,
                    self._erpm_kp,
                    self._erpm_ki,
                    self._erpm_integral_limit,
                    self._erpm_max_duty,
                )
                self._controlled_duty = slew_limit(
                    self._controlled_duty,
                    target_duty,
                    self._duty_slew_rate,
                    control_dt,
                )
                duty_cycle = self._controlled_duty

        startup_active = False
        if (not timed_out and duty_cycle != 0.0
                and self._startup_until is not None):
            measured_erpm = float(self._filtered_erpm or 0.0)
            direction = 1.0 if self._target_erpm > 0.0 else -1.0
            exit_threshold = min(
                self._startup_exit_erpm,
                abs(self._target_erpm) * self._startup_exit_target_ratio,
            )
            if direction * measured_erpm >= exit_threshold:
                if self._startup_confirm_since is None:
                    self._startup_confirm_since = now
            else:
                self._startup_confirm_since = None

            confirmation_elapsed = 0.0
            if self._startup_confirm_since is not None:
                confirmation_elapsed = (
                    now - self._startup_confirm_since).nanoseconds / 1e9
            startup_active = startup_boost_required(
                self._target_erpm,
                measured_erpm,
                now < self._startup_until,
                self._startup_exit_erpm,
                self._startup_exit_target_ratio,
                confirmation_elapsed,
                self._startup_exit_confirm,
            )
            if not startup_active:
                # Latch the completed state. The previous implementation
                # re-entered boost whenever noisy sensorless ERPM dipped below
                # the threshold, producing the recorded 12%-3.6% oscillation.
                self._startup_until = None
                self._startup_confirm_since = None
        if startup_active:
            # Breakaway must not inherit a larger PI request from a higher
            # mission speed.  The validated 8% command starts this sensorless
            # motor, while the previous minimum-only max() let a speed-8 PI
            # request climb back toward the current limit and cog on the
            # ground.  Keep both the published output and controller handoff
            # state at the fixed breakaway duty, and prevent integral windup.
            duty_cycle = fixed_startup_duty(
                self._target_erpm, self._startup_duty)
            self._controlled_duty = duty_cycle
            self._erpm_integral = self._erpm_startup_integral

        self._duty_publisher.publish(Float64(data=float(duty_cycle)))
        self._servo_publisher.publish(Float64(data=float(servo)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorCommandAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
