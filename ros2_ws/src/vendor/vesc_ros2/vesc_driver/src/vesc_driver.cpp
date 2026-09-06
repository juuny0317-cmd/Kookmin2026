// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_driver.hpp"

#include <vesc_msgs/msg/vesc_state.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <cassert>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <glob.h>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

namespace vesc_driver
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;
using sensor_msgs::msg::Imu;

VescDriver::VescDriver(const rclcpp::NodeOptions & options)
: rclcpp::Node("vesc_driver", options),
  vesc_(
    std::string(),
    std::bind(&VescDriver::vescPacketCallback, this, _1),
    std::bind(&VescDriver::vescErrorCallback, this, _1)),
  duty_cycle_limit_(this, "duty_cycle", -1.0, 1.0),
  current_limit_(this, "current"),
  brake_limit_(this, "brake"),
  speed_limit_(this, "speed"),
  position_limit_(this, "position"),
  servo_limit_(this, "servo", 0.0, 1.0),
  driver_mode_(MODE_DISCONNECTED),
  fw_version_major_(-1),
  fw_version_minor_(-1),
  legacy_protocol_(false),
  poll_imu_(false),
  firmware_compatible_(false),
  reconnect_requested_(false),
  last_telemetry_ns_(0),
  handshake_timeout_s_(2.0),
  reconnect_interval_s_(1.0),
  telemetry_timeout_s_(1.0),
  ready_(false)
{
  configured_port_ = declare_parameter<std::string>("port", "auto");
  auto_device_symlink_ =
    declare_parameter<std::string>("auto_device_symlink", "/dev/ttyMOTOR");
  auto_device_match_ =
    declare_parameter<std::string>("auto_device_match", "1a86_USB_Serial");
  ready_topic_ = declare_parameter<std::string>("ready_topic", "/vesc/ready");
  handshake_timeout_s_ = declare_parameter<double>("handshake_timeout_s", 2.0);
  reconnect_interval_s_ = declare_parameter<double>("reconnect_interval_s", 1.0);
  telemetry_timeout_s_ = declare_parameter<double>("telemetry_timeout_s", 1.0);
  poll_imu_ = declare_parameter<bool>("poll_imu", false);
  handshake_timeout_s_ = std::max(0.2, handshake_timeout_s_);
  reconnect_interval_s_ = std::max(0.1, reconnect_interval_s_);
  telemetry_timeout_s_ = std::max(0.2, telemetry_timeout_s_);

  // create vesc state (telemetry) publisher
  state_pub_ = create_publisher<VescStateStamped>("sensors/core", rclcpp::QoS{10});
  imu_pub_ = create_publisher<VescImuStamped>("sensors/imu", rclcpp::QoS{10});
  imu_std_pub_ = create_publisher<Imu>("sensors/imu/raw", rclcpp::QoS{10});
  ready_pub_ = create_publisher<std_msgs::msg::Bool>(
    ready_topic_, rclcpp::QoS(1).reliable().transient_local());

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ = create_publisher<Float64>(
    "sensors/servo_position_command", rclcpp::QoS{10});

  // subscribe to motor and servo command topics
  duty_cycle_sub_ = create_subscription<Float64>(
    "commands/motor/duty_cycle", rclcpp::QoS{10}, std::bind(
      &VescDriver::dutyCycleCallback, this,
      _1));
  current_sub_ = create_subscription<Float64>(
    "commands/motor/current", rclcpp::QoS{10}, std::bind(&VescDriver::currentCallback, this, _1));
  brake_sub_ = create_subscription<Float64>(
    "commands/motor/brake", rclcpp::QoS{10}, std::bind(&VescDriver::brakeCallback, this, _1));
  speed_sub_ = create_subscription<Float64>(
    "commands/motor/speed", rclcpp::QoS{10}, std::bind(&VescDriver::speedCallback, this, _1));
  position_sub_ = create_subscription<Float64>(
    "commands/motor/position", rclcpp::QoS{10}, std::bind(&VescDriver::positionCallback, this, _1));
  servo_sub_ = create_subscription<Float64>(
    "commands/servo/position", rclcpp::QoS{10}, std::bind(&VescDriver::servoCallback, this, _1));

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = create_wall_timer(20ms, std::bind(&VescDriver::timerCallback, this));
  next_reconnect_ = std::chrono::steady_clock::now();
  last_fw_request_ = next_reconnect_ - 1s;
  last_ready_publish_ = next_reconnect_ - 1s;
  publishReady(false, true);
  RCLCPP_INFO(
    get_logger(),
    "VESC driver started: port=%s, auto_symlink=%s, auto_match=%s, "
    "handshake_timeout=%.2fs, reconnect_interval=%.2fs, telemetry_timeout=%.2fs",
    configured_port_.c_str(), auto_device_symlink_.c_str(), auto_device_match_.c_str(),
    handshake_timeout_s_, reconnect_interval_s_, telemetry_timeout_s_);
}

VescDriver::~VescDriver()
{
  RCLCPP_INFO(get_logger(), "VESC shutdown requested.");
  publishReady(false, true);
  if (vesc_.isConnected()) {
    try {
      vesc_.setDutyCycleBlocking(0.0);
      RCLCPP_INFO(get_logger(), "VESC motor stop command sent (duty=0.0).");
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Could not send final motor stop command: %s", error.what());
    }
  }
  vesc_.disconnect();
  RCLCPP_INFO(get_logger(), "VESC reader stopped.");
  RCLCPP_INFO(get_logger(), "VESC asynchronous writer stopped.");
  RCLCPP_INFO(get_logger(), "VESC serial closed.");
  RCLCPP_INFO(get_logger(), "VESC shutdown complete.");
}

/* TODO or TO-THINKABOUT LIST
  - what should we do on startup? send brake or zero command?
  - what to do if the vesc interface gives an error?
  - check version number against know compatable?
  - should we wait until we receive telemetry before sending commands?
  - should we track the last motor command
  - what to do if no motor command received recently?
  - what to do if no servo command received recently?
  - what is the motor safe off state (0 current?)
  - what to do if a command parameter is out of range, ignore?
  - try to predict vesc bounds (from vesc config) and command detect bounds errors
*/

int64_t VescDriver::steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void VescDriver::publishReady(bool ready, bool force)
{
  const bool changed = ready_ != ready;
  if (!force && !changed) {
    return;
  }
  ready_ = ready;
  std_msgs::msg::Bool msg;
  msg.data = ready;
  ready_pub_->publish(msg);
  last_ready_publish_ = std::chrono::steady_clock::now();
  if (changed || !ready) {
    RCLCPP_INFO(get_logger(), "VESC readiness: %s", ready ? "READY" : "NOT_READY");
  }
}

std::string VescDriver::resolvePort(std::string & reason) const
{
  if (!configured_port_.empty() && configured_port_ != "auto") {
    if (access(configured_port_.c_str(), F_OK) == 0) {
      return configured_port_;
    }
    reason = "configured serial device does not exist: " + configured_port_;
    return std::string();
  }

  if (!auto_device_symlink_.empty() && access(auto_device_symlink_.c_str(), F_OK) == 0) {
    return auto_device_symlink_;
  }

  const auto glob_paths = [](const std::string & pattern) {
      std::vector<std::string> paths;
      glob_t matches{};
      if (glob(pattern.c_str(), 0, nullptr, &matches) == 0) {
        for (size_t i = 0; i < matches.gl_pathc; ++i) {
          paths.emplace_back(matches.gl_pathv[i]);
        }
      }
      globfree(&matches);
      std::sort(paths.begin(), paths.end());
      return paths;
    };

  std::vector<std::string> by_id_matches;
  for (const auto & path : glob_paths("/dev/serial/by-id/*")) {
    if (auto_device_match_.empty() || path.find(auto_device_match_) != std::string::npos) {
      by_id_matches.push_back(path);
    }
  }
  if (by_id_matches.size() == 1) {
    return by_id_matches.front();
  }
  if (by_id_matches.size() > 1) {
    reason = "multiple /dev/serial/by-id devices match '" + auto_device_match_ + "'";
    return std::string();
  }

  auto tty_usb = glob_paths("/dev/ttyUSB*");
  if (tty_usb.size() == 1) {
    RCLCPP_WARN(
      get_logger(),
      "No stable VESC device name matched; using the only /dev/ttyUSB device: %s",
      tty_usb.front().c_str());
    return tty_usb.front();
  }
  if (tty_usb.empty()) {
    reason = "no VESC candidate found (/dev/ttyMOTOR, matching by-id, or /dev/ttyUSB*)";
  } else {
    std::ostringstream message;
    message << "ambiguous /dev/ttyUSB candidates (";
    for (size_t i = 0; i < tty_usb.size(); ++i) {
      if (i != 0) {
        message << ", ";
      }
      message << tty_usb[i];
    }
    message << "); set port explicitly or provide a unique by-id match";
    reason = message.str();
  }
  return std::string();
}

bool VescDriver::connectIfAvailable()
{
  std::string reason;
  const auto port = resolvePort(reason);
  if (port.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[VESC] UART device not found or ambiguous: %s", reason.c_str());
    return false;
  }

  try {
    vesc_.connect(port);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(), "VESC serial open failed for %s: %s; retrying in %.2fs",
      port.c_str(), error.what(), reconnect_interval_s_);
    return false;
  }

  selected_port_ = port;
  firmware_compatible_.store(false);
  reconnect_requested_.store(false);
  fw_version_major_.store(-1);
  fw_version_minor_.store(-1);
  last_telemetry_ns_.store(0);
  driver_mode_.store(MODE_HANDSHAKING);
  handshake_started_ = std::chrono::steady_clock::now();
  last_fw_request_ = handshake_started_ - 1s;
  RCLCPP_INFO(
    get_logger(), "[VESC] Serial open succeeded on %s; waiting for firmware handshake.",
    port.c_str());
  return true;
}

void VescDriver::transitionToDisconnected(const std::string & reason)
{
  publishReady(false);
  driver_mode_.store(MODE_DISCONNECTED);
  reconnect_requested_.store(false);
  firmware_compatible_.store(false);
  last_telemetry_ns_.store(0);
  if (vesc_.isConnected()) {
    vesc_.disconnect();
  }
  next_reconnect_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(reconnect_interval_s_ * 1000.0));
  RCLCPP_WARN(
    get_logger(), "VESC disconnected (%s); reconnecting in %.2fs.",
    reason.c_str(), reconnect_interval_s_);
}

void VescDriver::timerCallback()
{
  const auto steady_now = std::chrono::steady_clock::now();
  const auto mode = driver_mode_.load();

  if (mode == MODE_DISCONNECTED) {
    if (steady_now >= next_reconnect_) {
      if (!connectIfAvailable()) {
        next_reconnect_ = steady_now +
          std::chrono::milliseconds(static_cast<int64_t>(reconnect_interval_s_ * 1000.0));
      }
    }
    return;
  }

  if (!vesc_.isConnected() || reconnect_requested_.load()) {
    transitionToDisconnected("serial receive/send failure");
    return;
  }

  if (mode == MODE_OPERATING && steady_now - last_ready_publish_ >= 500ms) {
    publishReady(true, true);
  }

  /*
   * Driver state machine, modes:
   *  HANDSHAKING - request and wait for VESC firmware version
   *  OPERATING - receiving commands from subscriber topics
   */
  try {
    if (mode == MODE_HANDSHAKING) {
      if (firmware_compatible_.load()) {
        const int major = fw_version_major_.load();
        const int minor = fw_version_minor_.load();
        driver_mode_.store(MODE_OPERATING);
        last_telemetry_ns_.store(steadyNowNs());
        publishReady(true);
        RCLCPP_INFO(
          get_logger(), "[VESC] Handshake succeeded; READY, firmware %d.%d on %s",
          major, minor, selected_port_.c_str());
        return;
      }

      const double handshake_age =
        std::chrono::duration<double>(steady_now - handshake_started_).count();
      if (handshake_age > handshake_timeout_s_) {
        RCLCPP_ERROR(
          get_logger(), "[VESC] Handshake timeout after %.2fs on %s.",
          handshake_timeout_s_, selected_port_.c_str());
        transitionToDisconnected("firmware handshake timeout");
        return;
      }
      if (steady_now - last_fw_request_ >= 100ms) {
        vesc_.requestFWVersion();
        last_fw_request_ = steady_now;
      }
    } else if (mode == MODE_OPERATING) {
      const int64_t telemetry_ns = last_telemetry_ns_.load();
      const double telemetry_age = telemetry_ns == 0 ? 0.0 :
        static_cast<double>(steadyNowNs() - telemetry_ns) / 1.0e9;
      if (telemetry_age > telemetry_timeout_s_) {
        transitionToDisconnected("telemetry timeout");
        return;
      }
      vesc_.requestState();
      if (poll_imu_ && !legacy_protocol_.load()) {
        vesc_.requestImuData();
      }
    } else {
      assert(false && "unknown driver mode");
    }
  } catch (const std::exception & error) {
    reconnect_requested_.store(true);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "VESC serial send failed: %s", error.what());
  }
}

void VescDriver::vescPacketCallback(const std::shared_ptr<VescPacket const> & packet)
{
  if (packet->name() == "Values") {
    std::shared_ptr<VescPacketValues const> values =
      std::dynamic_pointer_cast<VescPacketValues const>(packet);

    if (values->isLegacyLayout() != legacy_protocol_.load()) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejected VESC telemetry: firmware protocol and payload layout disagree "
        "(firmware=%d.%d, payload_size=%zu).",
        fw_version_major_.load(), fw_version_minor_.load(), values->payloadSize());
      return;
    }

    last_telemetry_ns_.store(steadyNowNs());

    auto state_msg = VescStateStamped();
    state_msg.header.stamp = now();

    state_msg.state.voltage_input = values->v_in();
    state_msg.state.current_motor = values->avg_motor_current();
    state_msg.state.current_input = values->avg_input_current();
    state_msg.state.avg_id = values->avg_id();
    state_msg.state.avg_iq = values->avg_iq();
    state_msg.state.duty_cycle = values->duty_cycle_now();
    state_msg.state.speed = values->rpm();

    state_msg.state.charge_drawn = values->amp_hours();
    state_msg.state.charge_regen = values->amp_hours_charged();
    state_msg.state.energy_drawn = values->watt_hours();
    state_msg.state.energy_regen = values->watt_hours_charged();
    state_msg.state.displacement = values->tachometer();
    state_msg.state.distance_traveled = values->tachometer_abs();
    state_msg.state.fault_code = values->fault_code();

    state_msg.state.pid_pos_now = values->pid_pos_now();
    state_msg.state.controller_id = values->controller_id();

    state_msg.state.ntc_temp_mos1 = values->temp_mos1();
    state_msg.state.ntc_temp_mos2 = values->temp_mos2();
    state_msg.state.ntc_temp_mos3 = values->temp_mos3();
    state_msg.state.avg_vd = values->avg_vd();
    state_msg.state.avg_vq = values->avg_vq();

    state_pub_->publish(state_msg);
  } else if (packet->name() == "FWVersion") {
    std::shared_ptr<VescPacketFWVersion const> fw_version =
      std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    const int major = fw_version->fwMajor();
    const int minor = fw_version->fwMinor();
    fw_version_major_.store(major);
    fw_version_minor_.store(minor);
    if (major == 2) {
      legacy_protocol_.store(true);
      vesc_.setLegacyProtocol(true);
      firmware_compatible_.store(true);
      RCLCPP_WARN(
        get_logger(),
        "Legacy VESC firmware %d.%d detected: using the 2.x telemetry layout, "
        "legacy servo command ID, and no IMU polling.",
        major, minor);
    } else if (major >= 5) {
      legacy_protocol_.store(false);
      vesc_.setLegacyProtocol(false);
      firmware_compatible_.store(true);
      RCLCPP_INFO(
        get_logger(), "Modern VESC firmware %d.%d detected on hardware '%s'.",
        major, minor, fw_version->hwname().c_str());
    } else if (major >= 0) {
      firmware_compatible_.store(false);
      reconnect_requested_.store(true);
      RCLCPP_ERROR(
        get_logger(),
        "Unsupported VESC firmware %d.%d. Refusing motor operation because "
        "the command layout is not verified.",
        major, minor);
    }
  } else if (packet->name() == "ImuData") {
    std::shared_ptr<VescPacketImu const> imuData =
      std::dynamic_pointer_cast<VescPacketImu const>(packet);

    auto imu_msg = VescImuStamped();
    auto std_imu_msg = Imu();
    imu_msg.header.stamp = now();
    std_imu_msg.header.stamp = now();

    imu_msg.imu.ypr.x = imuData->roll();
    imu_msg.imu.ypr.y = imuData->pitch();
    imu_msg.imu.ypr.z = imuData->yaw();

    imu_msg.imu.linear_acceleration.x = imuData->acc_x();
    imu_msg.imu.linear_acceleration.y = imuData->acc_y();
    imu_msg.imu.linear_acceleration.z = imuData->acc_z();

    imu_msg.imu.angular_velocity.x = imuData->gyr_x();
    imu_msg.imu.angular_velocity.y = imuData->gyr_y();
    imu_msg.imu.angular_velocity.z = imuData->gyr_z();

    imu_msg.imu.compass.x = imuData->mag_x();
    imu_msg.imu.compass.y = imuData->mag_y();
    imu_msg.imu.compass.z = imuData->mag_z();

    imu_msg.imu.orientation.w = imuData->q_w();
    imu_msg.imu.orientation.x = imuData->q_x();
    imu_msg.imu.orientation.y = imuData->q_y();
    imu_msg.imu.orientation.z = imuData->q_z();

    std_imu_msg.linear_acceleration.x = imuData->acc_x();
    std_imu_msg.linear_acceleration.y = imuData->acc_y();
    std_imu_msg.linear_acceleration.z = imuData->acc_z();

    std_imu_msg.angular_velocity.x = imuData->gyr_x();
    std_imu_msg.angular_velocity.y = imuData->gyr_y();
    std_imu_msg.angular_velocity.z = imuData->gyr_z();

    std_imu_msg.orientation.w = imuData->q_w();
    std_imu_msg.orientation.x = imuData->q_x();
    std_imu_msg.orientation.y = imuData->q_y();
    std_imu_msg.orientation.z = imuData->q_z();


    imu_pub_->publish(imu_msg);
    imu_std_pub_->publish(std_imu_msg);
  }
  auto & clk = *this->get_clock();
  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    clk,
    5000,
    "%s packet received",
    packet->name().c_str()
  );
}

void VescDriver::vescErrorCallback(const std::string & error)
{
  RCLCPP_ERROR(get_logger(), "%s", error.c_str());
  if (error.find("Serial receive failed:") == 0) {
    reconnect_requested_.store(true);
  }
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void VescDriver::dutyCycleCallback(const Float64::SharedPtr duty_cycle)
{
  if (driver_mode_.load() == MODE_OPERATING) {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
  }
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void VescDriver::currentCallback(const Float64::SharedPtr current)
{
  if (driver_mode_.load() == MODE_OPERATING) {
    vesc_.setCurrent(current_limit_.clip(current->data));
  }
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void VescDriver::brakeCallback(const Float64::SharedPtr brake)
{
  if (driver_mode_.load() == MODE_OPERATING) {
    vesc_.setBrake(brake_limit_.clip(brake->data));
  }
}

/**
 * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that the VESC may impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
void VescDriver::speedCallback(const Float64::SharedPtr speed)
{
  if (driver_mode_.load() == MODE_OPERATING) {
    vesc_.setSpeed(speed_limit_.clip(speed->data));
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriver::positionCallback(const Float64::SharedPtr position)
{
  if (driver_mode_.load() == MODE_OPERATING) {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
void VescDriver::servoCallback(const Float64::SharedPtr servo)
{
  if (driver_mode_.load() == MODE_OPERATING) {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    auto servo_sensor_msg = Float64();
    servo_sensor_msg.data = servo_clipped;
    servo_sensor_pub_->publish(servo_sensor_msg);
  }
}

VescDriver::CommandLimit::CommandLimit(
  rclcpp::Node * node_ptr,
  const std::string & str,
  const std::experimental::optional<double> & min_lower,
  const std::experimental::optional<double> & max_upper)
: node_ptr(node_ptr),
  logger(node_ptr->get_logger()),
  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  auto param_min =
    node_ptr->declare_parameter(name + "_min", rclcpp::ParameterValue(0.0));

  if (param_min.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    if (min_lower && param_min.get<double>() < *min_lower) {
      lower = *min_lower;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_min (" << param_min.get<double>() <<
          ") is less than the feasible minimum (" << *min_lower << ").");
    } else if (max_upper && param_min.get<double>() > *max_upper) {
      lower = *max_upper;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_min (" << param_min.get<double>() <<
          ") is greater than the feasible maximum (" << *max_upper << ").");
    } else {
      lower = param_min.get<double>();
    }
  } else if (min_lower) {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  auto param_max =
    node_ptr->declare_parameter(name + "_max", rclcpp::ParameterValue(0.0));

  if (param_max.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    if (min_lower && param_max.get<double>() < *min_lower) {
      upper = *min_lower;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_max (" << param_max.get<double>() <<
          ") is less than the feasible minimum (" << *min_lower << ").");
    } else if (max_upper && param_max.get<double>() > *max_upper) {
      upper = *max_upper;
      RCLCPP_WARN_STREAM(
        logger, "Parameter " << name << "_max (" << param_max.get<double>() <<
          ") is greater than the feasible maximum (" << *max_upper << ").");
    } else {
      upper = param_max.get<double>();
    }
  } else if (max_upper) {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper) {
    RCLCPP_WARN_STREAM(
      logger, "Parameter " << name << "_max (" << *upper <<
        ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";

  if (lower) {
    oss << *lower << " ";
  } else {
    oss << "(none) ";
  }

  if (upper) {
    oss << *upper;
  } else {
    oss << "(none)";
  }

  RCLCPP_DEBUG_STREAM(logger, oss.str());
}

double VescDriver::CommandLimit::clip(double value)
{
  auto clock = rclcpp::Clock(RCL_ROS_TIME);

  if (lower && value < lower) {
    RCLCPP_INFO_THROTTLE(
      logger, clock, 10, "%s command value (%f) below minimum limit (%f), clipping.",
      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper) {
    RCLCPP_INFO_THROTTLE(
      logger, clock, 10, "%s command value (%f) above maximum limit (%f), clipping.",
      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}

}  // namespace vesc_driver

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_driver::VescDriver)
