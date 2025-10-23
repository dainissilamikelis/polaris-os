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

#include "JoystickDemo.hpp"

#include <algorithm> // std::clamp()

namespace polaris_joystick {

using namespace ds_dbw_msgs::msg;

bool steerLpfCmd(uint8_t cmd_type) {
  switch (cmd_type) {
    case SteeringCmd::CMD_TORQUE:    return false;
    case SteeringCmd::CMD_ANGLE:     return true;
    case SteeringCmd::CMD_CURVATURE: return true;
    case SteeringCmd::CMD_YAW_RATE:  return true;
    case SteeringCmd::CMD_PERCENT:   return true;
    default: return false;
  }
}
float steerDefaultMax(uint8_t cmd_type) {
  switch (cmd_type) {
    case SteeringCmd::CMD_NONE:      return 0;
    case SteeringCmd::CMD_TORQUE:    return 8;     // Nm
    case SteeringCmd::CMD_ANGLE:     return 350;   // deg
    case SteeringCmd::CMD_CURVATURE: return 0.201; // 1/m
    case SteeringCmd::CMD_YAW_RATE:  return 5.01;  // rad/s
    case SteeringCmd::CMD_PERCENT:   return 100.1; // %
    default: return 0;
  }
}
float brakeDefaultMin(uint8_t cmd_type) {
  switch (cmd_type) {
    case BrakeCmd::CMD_NONE:      return 0;
    case BrakeCmd::CMD_PRESSURE:  return 0;   // bar
    case BrakeCmd::CMD_TORQUE:    return 0;   // Nm
    case BrakeCmd::CMD_ACCEL:     return 0.1; // m/s^2
    case BrakeCmd::CMD_ACCEL_ACC: return 0.1; // m/s^2
    case BrakeCmd::CMD_ACCEL_AEB: return 0.1; // m/s^2
    case BrakeCmd::CMD_PEDAL_RAW: return 10;  // %
    case BrakeCmd::CMD_PERCENT:   return 0;   // %
    default: return 0;
  }
}
float brakeDefaultMax(uint8_t cmd_type) {
  switch (cmd_type) {
    case BrakeCmd::CMD_NONE:      return 0;
    case BrakeCmd::CMD_PRESSURE:  return 100;  // bar
    case BrakeCmd::CMD_TORQUE:    return 5000; // Nm
    case BrakeCmd::CMD_ACCEL:     return -8.0; // m/s^2
    case BrakeCmd::CMD_ACCEL_ACC: return -8.0; // m/s^2
    case BrakeCmd::CMD_ACCEL_AEB: return -8.0; // m/s^2
    case BrakeCmd::CMD_PEDAL_RAW: return  90;  // %
    case BrakeCmd::CMD_PERCENT:   return  80;  // %
    default: return 0;
  }
}
float thrtlDefaultMin(uint8_t cmd_type) {
  switch (cmd_type) {
    case ThrottleCmd::CMD_NONE:      return 0;
    case ThrottleCmd::CMD_PEDAL_RAW: return 10; // %
    case ThrottleCmd::CMD_PERCENT:   return 0;  // %
    default: return 0;
  }
}
float thrtlDefaultMax(uint8_t cmd_type) {
  switch (cmd_type) {
    case ThrottleCmd::CMD_NONE:      return 0;
    case ThrottleCmd::CMD_PEDAL_RAW: return 90;    // %
    case ThrottleCmd::CMD_PERCENT:   return 100.1; // %
    default: return 0;
  }
}

JoystickDemo::JoystickDemo(const rclcpp::NodeOptions &options) : rclcpp::Node("joy_polaris", options) {
  startup_stamp_ = ros_clock_.now();

  joy_.axes.resize(AXIS_COUNT_X, 0);
  joy_.buttons.resize(BTN_COUNT_X, 0);

  enable_ = declare_parameter("enable", enable_);
  ignore_ = declare_parameter("ignore", ignore_);

  steer_ = declare_parameter("steer", steer_);
  brake_ = declare_parameter("brake", brake_);
  thrtl_ = declare_parameter("thrtl", thrtl_);
  shift_ = declare_parameter("shift", shift_);
  misc_ = declare_parameter("misc", misc_);

  steer_cmd_type_ = steerCmdType(declare_parameter("steer_cmd_type", ""));
  brake_cmd_type_ = brakeCmdType(declare_parameter("brake_cmd_type", ""));
  thrtl_cmd_type_ = thrtlCmdType(declare_parameter("thrtl_cmd_type", ""));

  steer_max_ = steerDefaultMax(steer_cmd_type_);
  brake_min_ = brakeDefaultMin(brake_cmd_type_);
  brake_max_ = brakeDefaultMax(brake_cmd_type_);
  thrtl_min_ = thrtlDefaultMin(thrtl_cmd_type_);
  thrtl_max_ = thrtlDefaultMax(thrtl_cmd_type_);
  try { steer_max_ = declare_parameter("steer_max", steer_max_); } catch (...) {}
  try { brake_max_ = declare_parameter("brake_max", brake_max_); } catch (...) {}
  try { brake_min_ = declare_parameter("brake_min", brake_min_); } catch (...) {}
  try { thrtl_max_ = declare_parameter("thrtl_max", thrtl_max_); } catch (...) {}
  try { thrtl_min_ = declare_parameter("thrtl_min", thrtl_min_); } catch (...) {}

  steer_rate_ = declare_parameter("steer_rate", steer_rate_);
  steer_accel_ = declare_parameter("steer_accel", steer_accel_);
  brake_inc_ = declare_parameter("brake_inc", brake_inc_);
  brake_dec_ = declare_parameter("brake_dec", brake_dec_);
  thrtl_inc_ = declare_parameter("thrtl_inc", thrtl_inc_);
  thrtl_dec_ = declare_parameter("thrtl_dec", thrtl_dec_);

  using std::placeholders::_1;
  sub_joy_ = create_subscription<sensor_msgs::msg::Joy>("/joy_polaris", 1, std::bind(&JoystickDemo::recvJoy, this, _1));
  sub_veh_vel_ = create_subscription<VehicleVelocity>("/vehicle/vehicle_velocity", 1, std::bind(&JoystickDemo::recvVehVel, this, _1));

  data_.brake_joy = 0.0;
  data_.gear_cmd = Gear::NONE;
  data_.steering_joy = 0.0;
  data_.steering_mult = true;
  data_.steering_cal = false;
  data_.throttle_joy = 0.0;
  data_.joy_throttle_valid = false;
  data_.joy_brake_valid = false;

  if (steer_) {
    pub_steer_ = create_publisher<SteeringCmd>("steering/cmd", 1);
  }
  if (brake_) {
    pub_brake_ = create_publisher<BrakeCmd>("brake/cmd", 1);
  }
  if (thrtl_) {
    pub_thrtl_ = create_publisher<ThrottleCmd>("throttle/cmd", 1);
  }
  if (shift_) {
    pub_gear_ = create_publisher<GearCmd>("gear/cmd", 1);
    sub_gear_ = create_subscription<GearReport>("gear/report", 1, std::bind(&JoystickDemo::recvGear, this, _1));
  }
  if (misc_) {
    pub_turn_signal_ = create_publisher<TurnSignalCmd>("turn_signal/cmd", 1);
    #if 0
    pub_misc_ = create_publisher<MiscCmd>("misc/cmd", 1);
    #endif
  }
  if (enable_) {
    pub_enable_ = create_publisher<std_msgs::msg::Empty>("enable", 1);
    pub_disable_ = create_publisher<std_msgs::msg::Empty>("disable", 1);
  }

  // Initilize timestamp to be old (timeout)
  using namespace std::chrono_literals;
  data_.stamp = now() - 1s;
  timer_ = create_wall_timer(20ms, std::bind(&JoystickDemo::cmdCallback, this));
}

void JoystickDemo::cmdCallback() {
  using namespace std::chrono_literals;
  bool startup_finished = ros_clock_.now() - startup_stamp_ > 1s;

  // Detect joy timeouts and reset
  if (now() - data_.stamp > 100ms) {
    data_.joy_throttle_valid = false;
    data_.joy_brake_valid = false;
    last_steering_filt_output_ = 0.0;
    if (!joy_warned_ && startup_finished) {
      joy_warned_ = true;
      RCLCPP_WARN(get_logger(), "Game controller timeout");
    }
    return; // Game controller timeout
  } else {
    if (joy_warned_) {
      joy_warned_ = false;
      RCLCPP_INFO(get_logger(), "Game controller ready");
    }
  }

  // Vehicle velocity
  float velocity = getVehVel();
  constexpr float KPH_TO_MPS = 1 / 3.6;
  if (std::isfinite(velocity)) {
    if (velocity_warned_) {
      velocity_warned_ = false;
      RCLCPP_INFO(get_logger(), "Vehicle velocity ready");
    }
  } else {
    if (!velocity_warned_ && startup_finished) {
      velocity_warned_ = true;
      RCLCPP_WARN(get_logger(), "Waiting for vehicle velocity");
    }
    return; // Invalid vehicle velocity or timeout
  }

  // Steering
  if (steer_) {
    if (std::abs(velocity) > 45 * KPH_TO_MPS) { // 45 kph
      if (!disallow_steer_cmd_) {
        disallow_steer_cmd_ = true;
        RCLCPP_WARN(get_logger(), "Disabling steering commands due to excessive vehicle velocity");
      }
    } else if (std::abs(velocity) < 10 * KPH_TO_MPS) { // 10 kph
      if (disallow_steer_cmd_) {
        disallow_steer_cmd_ = false;
        RCLCPP_INFO(get_logger(), "Re-enabling steering commands at lower vehicle velocity");
      }
    }
    if (!disallow_steer_cmd_) {
      SteeringCmd msg;
      msg.enable = true;
      msg.ignore = ignore_;
      if (data_.steering_cal) {
        msg.cmd_type = SteeringCmd::CMD_CALIBRATE;
        msg.cmd = 0.0;
      } else {
        msg.cmd_type = steer_cmd_type_;
        msg.cmd_rate  = steer_rate_;
        msg.cmd_accel = steer_accel_;
        float steering_joy = data_.steering_joy;
        if (!data_.steering_mult) {
          steering_joy *= 0.98;
        }
        if (steerLpfCmd(steer_cmd_type_)) {
          //constexpr float TAU = 0.01;
          constexpr float TAU = 0.05;
          //float filtered_steering_cmd = 0.02f / TAU * steering_joy + (1.0f - 0.02f / TAU) * last_steering_filt_output_;
          float filtered_steering_cmd = 0.01f / TAU * steering_joy + (1.0f - 0.01f / TAU) * last_steering_filt_output_;
          last_steering_filt_output_ = filtered_steering_cmd;
          msg.cmd = filtered_steering_cmd * steer_max_;
        } else {
          msg.cmd = steering_joy * steer_max_;
        }
      }
      pub_steer_->publish(msg);
    }
  }

  // Brake
  if (brake_) {
    BrakeCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    msg.cmd_type = brake_cmd_type_;
    msg.cmd = data_.brake_joy * (brake_max_ - brake_min_) + brake_min_;
    msg.rate_inc = brake_inc_;
    msg.rate_dec = brake_dec_;
    msg.precharge_aeb = data_.brake_precharge;
    pub_brake_->publish(msg);
  }

  // Throttle
  if (thrtl_) {
    ThrottleCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    msg.cmd_type = thrtl_cmd_type_;
    msg.cmd = data_.throttle_joy * (thrtl_max_ - thrtl_min_) + thrtl_min_;
    msg.rate_inc = thrtl_inc_;
    msg.rate_dec = thrtl_dec_;
    pub_thrtl_->publish(msg);
  }

  // Gear
  if (shift_) {
    if (data_.gear_cmd != Gear::NONE) {
      uint8_t gear = getGear();
      if (gear != Gear::NONE) {
        GearCmd msg;
        msg.cmd.value = data_.gear_cmd;
        switch (msg.cmd.value) {
          case Gear::PARK:
            if (gear == Gear::PARK || std::abs(velocity) < 10 * KPH_TO_MPS) {
              pub_gear_->publish(msg);
            } else {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2e3, "Not sending gear command park: Excessive vehicle velocity");
            }
            break;
          case Gear::REVERSE:
            if (gear == Gear::REVERSE || velocity < 10 * KPH_TO_MPS) {
              pub_gear_->publish(msg);
            } else {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2e3, "Not sending gear command reverse: Excessive vehicle velocity");
            }
            break;
          case Gear::NEUTRAL:
            pub_gear_->publish(msg);
            break;
          case Gear::DRIVE: // HIGH DRIVE
          case Gear::LOW:
            if (gear == Gear::DRIVE || gear == Gear::LOW || velocity > -10 * KPH_TO_MPS) {
              pub_gear_->publish(msg);
            } else {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2e3, "Not sending gear command drive: Excessive vehicle velocity");
            }
            break;
          default:
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2e3, "Not sending unknown gear command: %u", msg.cmd.value);
            break;
        }
      }
    }
  }

  // Turn signal
  if (misc_) {
    TurnSignalCmd msg_turn_signal;
    msg_turn_signal.cmd.value = data_.turn_signal_cmd;
    pub_turn_signal_->publish(msg_turn_signal);

    #if 0
    MiscCmd msg_misc;
    // msg_misc.parking_brake.value = 0;
    // msg_misc.door.select = data_.door_select;
    // msg_misc.door.action = data_.door_action;
    pub_misc_->publish(msg_misc);
    #endif
  }
}

void JoystickDemo::recvJoy(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
  // Check for expected sizes
  if (msg->axes.size() != (size_t)AXIS_COUNT_X && msg->buttons.size() != (size_t)BTN_COUNT_X) {
    if (msg->axes.size() == (size_t)AXIS_COUNT_D && msg->buttons.size() == (size_t)BTN_COUNT_D) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2e3,
          "Detected Logitech Gamepad F310 in DirectInput (D) mode. Please select (Start) with the switch on "
          "the back to select XInput mode.");
    }
    if (msg->axes.size() != (size_t)AXIS_COUNT_X) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2e3, "Expected %zu joy axis count, received %zu",
                            (size_t)AXIS_COUNT_X, msg->axes.size());
    }
    if (msg->buttons.size() != (size_t)BTN_COUNT_X) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2e3, "Expected %zu joy button count, received %zu",
                            (size_t)BTN_COUNT_X, msg->buttons.size());
    }
    return;
  }

  // Handle joystick startup
  if (msg->axes[AXIS_THROTTLE] != 0.0) {
    data_.joy_throttle_valid = true;
  }
  if (msg->axes[AXIS_BRAKE] != 0.0) {
    data_.joy_brake_valid = true;
  }

  // Throttle
  if (data_.joy_throttle_valid) {
    data_.throttle_joy = 0.5 - 0.5 * msg->axes[AXIS_THROTTLE];
  }

  // Brake
  if (data_.joy_brake_valid) {
    data_.brake_joy = 0.5 - 0.5 * msg->axes[AXIS_BRAKE];
  }
  if ((msg->axes[AXIS_BRAKE_PRECHARGE] < -0.5) && (msg->axes[AXIS_STEER_MULT_1] == 0.0)) { // avoiding double press
    data_.brake_precharge = ds_dbw_msgs::msg::BrakeCmd::PRECHARGE_LEVEL_2;
  } else if ((msg->axes[AXIS_BRAKE_PRECHARGE] > 0.5) && (msg->axes[AXIS_STEER_MULT_1] == 0.0)){ // avoiding double press
    data_.brake_precharge = ds_dbw_msgs::msg::BrakeCmd::PRECHARGE_LEVEL_1;
  } else {
    data_.brake_precharge = ds_dbw_msgs::msg::BrakeCmd::PRECHARGE_NONE;
  }

  // Gear
  if (msg->buttons[BTN_PARK]) {
    data_.gear_cmd = Gear::PARK;
  } else if (msg->buttons[BTN_REVERSE]) {
    data_.gear_cmd = Gear::REVERSE;
  } else if (msg->buttons[BTN_DRIVE]) {
    data_.gear_cmd = Gear::DRIVE;
  } else if (msg->buttons[BTN_NEUTRAL]) {
    data_.gear_cmd = Gear::NEUTRAL;
  } else if (msg->buttons[BTN_LOW]) {
    data_.gear_cmd = Gear::LOW;
  } else if (msg->buttons[BTN_WD]) { // must be UPDATED !!!
    // RESERVED for the update
    data_.gear_cmd = Gear::LOW;
  }
  else {
    data_.gear_cmd = Gear::NONE;
  }

  // Steering
  if(std::abs(msg->axes[AXIS_STEER_1]) > std::abs(msg->axes[AXIS_STEER_2])) {
    data_.steering_joy = msg->axes[AXIS_STEER_1];
  } else {
    data_.steering_joy = msg->axes[AXIS_STEER_2];
  }
  data_.steering_mult = static_cast<bool>(msg->axes[AXIS_STEER_MULT_1]) && !static_cast<bool>(msg->axes[AXIS_STEER_MULT_2]);
  data_.steering_cal = static_cast<bool>(std::abs(msg->axes[AXIS_STEER_MULT_1] * -1)) && static_cast<bool>(std::abs(msg->axes[AXIS_STEER_MULT_2] * -1)); // double press

  // Turn signal
  if (msg->axes[AXIS_TURN_SIG] != joy_.axes[AXIS_TURN_SIG]) {
    if ((msg->axes[AXIS_TURN_SIG] != 0.0) && (msg->axes[AXIS_STEER_MULT_1] == 0.0)) {
      switch (data_.turn_signal_cmd) {
        case TurnSignal::NONE:
          if (msg->axes[AXIS_TURN_SIG] < -0.5) {
            data_.turn_signal_cmd = TurnSignal::RIGHT;
          } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
            data_.turn_signal_cmd = TurnSignal::LEFT;
          }
          break;
        case TurnSignal::LEFT:
          if (msg->axes[AXIS_TURN_SIG] < -0.5) {
            data_.turn_signal_cmd = TurnSignal::RIGHT;
          } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
            data_.turn_signal_cmd = TurnSignal::NONE;
          }
          break;
        case TurnSignal::RIGHT:
          if (msg->axes[AXIS_TURN_SIG] < -0.5) {
            data_.turn_signal_cmd = TurnSignal::NONE;
          } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
            data_.turn_signal_cmd = TurnSignal::LEFT;
          }
          break;
      }
    }
  }

  // #if 0
  // // Doors and trunk
  // data_.door_select = DoorCmd::NONE;
  // data_.door_action = DoorCmd::NONE;
  // if (msg->buttons[BTN_TRUNK_OPEN]) {
  //   data_.door_select = DoorCmd::TRUNK;
  //   data_.door_action = DoorCmd::OPEN;
  // } else if (msg->buttons[BTN_TRUNK_CLOSE]) {
  //   data_.door_select = DoorCmd::TRUNK;
  //   data_.door_action = DoorCmd::CLOSE;
  // }
  // if (msg->axes[AXIS_DOOR_ACTION] > 0.5) {
  //   if (msg->axes[AXIS_DOOR_SELECT] < -0.5) {
  //     data_.door_select = DoorCmd::RIGHT;
  //     data_.door_action = DoorCmd::OPEN;
  //   } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
  //     data_.door_select = DoorCmd::LEFT;
  //     data_.door_action = DoorCmd::OPEN;
  //   }
  // }
  // if (msg->axes[AXIS_DOOR_ACTION] < -0.5) {
  //   if (msg->axes[AXIS_DOOR_SELECT] < -0.5) {
  //     data_.door_select = DoorCmd::RIGHT;
  //     data_.door_action = DoorCmd::CLOSE;
  //   } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
  //     data_.door_select = DoorCmd::LEFT;
  //     data_.door_action = DoorCmd::CLOSE;
  //   }
  // }
  // #endif

  // Optional enable and disable buttons
  if (enable_) {
    if (msg->buttons[BTN_ENABLE]) {
      pub_enable_->publish(std_msgs::msg::Empty());
    }
    if (!msg->buttons[BTN_ENABLE]) { // BTN_DISTABLE, redefined
      pub_disable_->publish(std_msgs::msg::Empty());
    }
  }

  data_.stamp = now();
  joy_ = *msg;
}

void JoystickDemo::recvGear(const GearReport::ConstSharedPtr msg) {
  msg_gear_ = *msg;
  msg_gear_stamp_ = ros_clock_.now();
}

void JoystickDemo::recvVehVel(const VehicleVelocity::ConstSharedPtr msg) {
  veh_vel_ = *msg;
  veh_vel_stamp_ = ros_clock_.now();
}

} // namespace polaris_joystick

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(polaris_joystick::JoystickDemo)
