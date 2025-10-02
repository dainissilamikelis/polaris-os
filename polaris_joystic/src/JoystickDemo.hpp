/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020-2021, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>

#include <ds_dbw_msgs/msg/steering_cmd.hpp>
#include <ds_dbw_msgs/msg/brake_cmd.hpp>
#include <ds_dbw_msgs/msg/throttle_cmd.hpp>
#include <ds_dbw_msgs/msg/gear_cmd.hpp>
#include <ds_dbw_msgs/msg/turn_signal_cmd.hpp>
#include <ds_dbw_msgs/msg/misc_cmd.hpp>
#include <ds_dbw_msgs/msg/gear_report.hpp>
#include <ds_dbw_msgs/msg/vehicle_velocity.hpp>

namespace ds_dbw_joystick_demo {

class JoystickDemo : public rclcpp::Node {
public:
  JoystickDemo(const rclcpp::NodeOptions& options);

private:
  void recvJoy(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void recvGear(const ds_dbw_msgs::msg::GearReport::ConstSharedPtr msg);
  void recvVehVel(const ds_dbw_msgs::msg::VehicleVelocity::ConstSharedPtr msg);
  void cmdCallback();

  // Topics
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<ds_dbw_msgs::msg::GearReport>::SharedPtr sub_gear_;
  rclcpp::Subscription<ds_dbw_msgs::msg::VehicleVelocity>::SharedPtr sub_veh_vel_;
  rclcpp::Publisher<ds_dbw_msgs::msg::SteeringCmd>::SharedPtr pub_steer_;
  rclcpp::Publisher<ds_dbw_msgs::msg::BrakeCmd>::SharedPtr pub_brake_;
  rclcpp::Publisher<ds_dbw_msgs::msg::ThrottleCmd>::SharedPtr pub_thrtl_;
  rclcpp::Publisher<ds_dbw_msgs::msg::GearCmd>::SharedPtr pub_gear_;
  rclcpp::Publisher<ds_dbw_msgs::msg::TurnSignalCmd>::SharedPtr pub_turn_signal_;
  #if 0
  rclcpp::Publisher<ds_dbw_msgs::msg::MiscCmd>::SharedPtr pub_misc_;
  #endif
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_enable_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_disable_;

  // Parameters
  bool enable_ = true;  // Use enable and disable buttons
  bool ignore_ = false; // Ignore driver overrides

  // Parameters
  bool steer_ = true; // Send steering commands
  bool brake_ = true; // Send brake commands
  bool thrtl_ = true; // Send throttle commands
  bool shift_ = true; // Send shift commands
  bool misc_ = true;  // Send misc commands

  // Parameters
  uint8_t steer_cmd_type_ = ds_dbw_msgs::msg::SteeringCmd::CMD_NONE;
  uint8_t brake_cmd_type_ = ds_dbw_msgs::msg::BrakeCmd::CMD_NONE;
  uint8_t thrtl_cmd_type_ = ds_dbw_msgs::msg::ThrottleCmd::CMD_NONE;

  // Parameters
  float steer_max_ = 0; // Maximum steer command (changes with cmd_type)
  float brake_min_ = 0; // Minimum brake command (changes with cmd_type)
  float brake_max_ = 0; // Maximum brake command (changes with cmd_type)
  float thrtl_min_ = 0; // Minimum thrtl command (changes with cmd_type)
  float thrtl_max_ = 0; // Maximum thrtl command (changes with cmd_type)

  // Parameters
  float steer_rate_ = 0;  // Rate limit: deg/s
  float steer_accel_ = 0; // Accel limit deg/s^2
  float brake_inc_ = 0;   // Rate increase limit (changes with cmd_type)
  float brake_dec_ = 0;   // Rate decrease limit (changes with cmd_type)
  float thrtl_inc_ = 0;   // Rate increase limit %/s
  float thrtl_dec_ = 0;   // Rate decrease limit %/s

  // Command type to string
  uint8_t steerCmdType(const std::string &str) {
    if (!str.empty()) {
      if (str == "none")      return ds_dbw_msgs::msg::SteeringCmd::CMD_NONE;
      if (str == "torque")    return ds_dbw_msgs::msg::SteeringCmd::CMD_TORQUE;
      if (str == "angle")     return ds_dbw_msgs::msg::SteeringCmd::CMD_ANGLE;
      if (str == "curvature") return ds_dbw_msgs::msg::SteeringCmd::CMD_CURVATURE;
      if (str == "yaw_rate")  return ds_dbw_msgs::msg::SteeringCmd::CMD_YAW_RATE;
      if (str == "percent")   return ds_dbw_msgs::msg::SteeringCmd::CMD_PERCENT;
      RCLCPP_ERROR(get_logger(), "Unknown steer_cmd_type: %s", str.c_str());
      return ds_dbw_msgs::msg::SteeringCmd::CMD_NONE;
    }
    return ds_dbw_msgs::msg::SteeringCmd::CMD_PERCENT;
  }
  uint8_t brakeCmdType(const std::string &str) {
    if (!str.empty()) {
      if (str == "none")      return ds_dbw_msgs::msg::BrakeCmd::CMD_NONE;
      if (str == "pressure")  return ds_dbw_msgs::msg::BrakeCmd::CMD_PRESSURE;
      if (str == "torque")    return ds_dbw_msgs::msg::BrakeCmd::CMD_TORQUE;
      if (str == "accel")     return ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL;
      if (str == "accel_acc") return ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL_ACC;
      if (str == "accel_aeb") return ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL_AEB;
      if (str == "pedal_raw") return ds_dbw_msgs::msg::BrakeCmd::CMD_PEDAL_RAW;
      if (str == "percent")   return ds_dbw_msgs::msg::BrakeCmd::CMD_PERCENT;
      RCLCPP_ERROR(get_logger(), "Unknown brake_cmd_type: %s", str.c_str());
      return ds_dbw_msgs::msg::BrakeCmd::CMD_NONE;
    }
    return ds_dbw_msgs::msg::BrakeCmd::CMD_PERCENT;
  }
  uint8_t thrtlCmdType(const std::string &str) {
    if (!str.empty()) {
      if (str == "none")      return ds_dbw_msgs::msg::ThrottleCmd::CMD_NONE;
      if (str == "pedal_raw") return ds_dbw_msgs::msg::ThrottleCmd::CMD_PEDAL_RAW;
      if (str == "percent")   return ds_dbw_msgs::msg::ThrottleCmd::CMD_PERCENT;
      RCLCPP_ERROR(get_logger(), "Unknown thrtl_cmd_type: %s", str.c_str());
      return ds_dbw_msgs::msg::ThrottleCmd::CMD_NONE;
    }
    return ds_dbw_msgs::msg::ThrottleCmd::CMD_PERCENT;
  }

  // Clock for received message timestamps
  rclcpp::Clock ros_clock_ = rclcpp::Clock(RCL_ROS_TIME);
  rclcpp::Time startup_stamp_;

  // Gear report
  rclcpp::Time msg_gear_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  ds_dbw_msgs::msg::GearReport msg_gear_;
  uint8_t getGear() {
    using namespace std::chrono_literals;
    if (ros_clock_.now() - msg_gear_stamp_ < 500ms) {
      return msg_gear_.gear.value;
    }
    return ds_dbw_msgs::msg::Gear::NONE;
  }

  // Vehicle velocity
  bool disallow_steer_cmd_ = false;
  bool velocity_warned_ = false;
  rclcpp::Time veh_vel_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  ds_dbw_msgs::msg::VehicleVelocity veh_vel_;
  static float select2(float a, float b) { return !std::isnan(a) ? a : b; }
  float getVehVel() {
    using namespace std::chrono_literals;
    if (ros_clock_.now() - veh_vel_stamp_ < 500ms) {
      return select2(veh_vel_.vehicle_velocity_brake, veh_vel_.vehicle_velocity_propulsion);
    }
    return NAN;
  }

  // Variables
  struct {
    rclcpp::Time stamp;
    float brake_joy = 0;
    float throttle_joy = 0;
    float steering_joy = 0;
    uint8_t gear_cmd = 0;
    uint8_t turn_signal_cmd = 0;
    uint8_t door_select = 0;
    uint8_t door_action = 0;
    uint8_t brake_precharge = 0;
    bool steering_mult = false;
    bool steering_cal = false;
    bool joy_throttle_valid = false;
    bool joy_brake_valid = false;
  } data_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy joy_;
  float last_steering_filt_output_ = 0;
  bool joy_warned_ = false;

  enum {
    BTN_PARK = 3,
    BTN_REVERSE = 1,
    BTN_NEUTRAL = 2,
    BTN_DRIVE = 0,
    BTN_ENABLE = 5,
    BTN_DISABLE = 4,
    BTN_STEER_MULT_1 = 6,
    BTN_STEER_MULT_2 = 7,
    BTN_TRUNK_OPEN = 9,
    BTN_TRUNK_CLOSE = 10,
    BTN_COUNT_X = 11,
    BTN_COUNT_D = 12,
    AXIS_THROTTLE = 5,
    AXIS_BRAKE = 2,
    AXIS_STEER_1 = 0,
    AXIS_STEER_2 = 3,
    AXIS_TURN_SIG = 6,
    AXIS_BRAKE_PRECHARGE = 7,
    AXIS_DOOR_SELECT = 6,
    AXIS_DOOR_ACTION = 7,
    AXIS_COUNT_D = 6,
    AXIS_COUNT_X = 8,
  };
};

} // namespace ds_dbw_joystick_demo
