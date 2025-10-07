#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/mavlink.hpp"
#include <mavlink/v2.0/common/mavlink.h>
#include "ds_dbw_msgs/msg/gear_report.hpp"
#include "ds_dbw_msgs/msg/steering_report.hpp"
#include "ds_dbw_msgs/msg/fuel_level.hpp"
#include "ds_dbw_msgs/msg/vehicle_velocity.hpp"
#include "ds_dbw_msgs/msg/system_report.hpp"
#include "ds_dbw_msgs/msg/remote_report.hpp"
#include "ds_dbw_msgs/msg/drive_mode_report.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

class UGVTelemetry : public rclcpp::Node {
public:
  UGVTelemetry() : Node("ugv_telemetry") {

    // subscribe UGV FUEL report
    fuel_sub_ = this->create_subscription<ds_dbw_msgs::msg::FuelLevel>(
      "/vehicle/fuel_level", 1,
      std::bind(&UGVTelemetry::fuel_callback, this, std::placeholders::_1)
    );

    // subscribe UGV current velocity
    velocity_sub_ = this->create_subscription<ds_dbw_msgs::msg::VehicleVelocity>(
      "/vehicle/vehicle_velocity", 1,
      std::bind(&UGVTelemetry::velocity_callback, this, std::placeholders::_1)
    );

    // subscribe UGV system's status
    sys_report_sub_ = this->create_subscription<ds_dbw_msgs::msg::SystemReport>(
      "/vehicle/system/report", 1,
      std::bind(&UGVTelemetry::system_report_callback, this, std::placeholders::_1)
    );

    // subscribe UGV remote report
    rem_report_sub_ = this->create_subscription<ds_dbw_msgs::msg::RemoteReport>(
      "/vehicle/remote/report", 1,
      std::bind(&UGVTelemetry::remote_report_callback, this, std::placeholders::_1)
    );

    // subscribe UGV drive mode status
    drive_mode_report_sub_ = this->create_subscription<ds_dbw_msgs::msg::DriveModeReport>(
      "/vehicle/drive_mode/report", 1,
      std::bind(&UGVTelemetry::drive_mode_report_callback, this, std::placeholders::_1)
    );

    // subscribe GPS info
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/vehicle/gps/fix", 1,
      std::bind(&UGVTelemetry::gps_callback, this, std::placeholders::_1)
    );
    
    // subscribe Steering info
    steering_sub_ = this->create_subscription<ds_dbw_msgs::msg::SteeringReport>(
      "/vehicle/steering_report", 1,
      std::bind(&UGVTelemetry::steering_report_callback, this, std::placeholders::_1));

    // subscribe Gear info
    gear_sub_ = this->create_subscription<ds_dbw_msgs::msg::GearReport>(
      "/vehicle/gear_report", 1,
      std::bind(&UGVTelemetry::gear_report_callback, this, std::placeholders::_1));

    // manual_pub_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
    //   "/mavros/manual_control/send", 10
    // );
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&UGVTelemetry::check_subs_, this));

    RCLCPP_INFO(this->get_logger(), "MAVROS UGVTelemetry node started.");

    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
        RCLCPP_INFO(this->get_logger(), "Socket creation failed.");
    }

    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(14550);  // Target port (e.g. QGroundControl or autopilot)
    remote_addr_.sin_addr.s_addr = inet_addr("172.25.65.12");  // Target IP / local IP = 172.25.65.12, Remote IP = 172.25.64.197
  }

private:
  void fuel_callback(const ds_dbw_msgs::msg::FuelLevel::SharedPtr msg) {
    // reading messages from UGV about the fuel level

    float fuel_level = msg->fuel_level;
    float fuel_range = msg->fuel_range;
    float odometry = msg->odometer;
  }

  void check_subs_() {
    // checks subscribed topics are still arriving

    if (steering_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/steering_report' msgs published ('%s')?", this->get_name());
    }

    if (gear_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/gear_report' msgs published ('%s')?", this->get_name());
    }

    if (fuel_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/fuel_level' msgs published ('%s')?", this->get_name());
    }

    if (velocity_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/vehicle_velocity' msgs published ('%s')?", this->get_name());
    }   

    if (sys_report_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/system/report' msgs published ('%s')?", this->get_name());
    } 

    if (rem_report_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/remote/report' msgs published ('%s')?", this->get_name());
    }

    if (drive_mode_report_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/drive_mode/report' msgs published ('%s')?", this->get_name());
    }

    if (gps_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/gps/fix' msgs published ('%s')?", this->get_name());
    } 
    
  }

  void gear_report_callback(const ds_dbw_msgs::msg::GearReport::SharedPtr msg) {
  // reading messages from UGV about gear
  }

  void steering_report_callback(const ds_dbw_msgs::msg::SteeringReport::SharedPtr msg) {
  // reading messages from UGV about steering
  }

  void velocity_callback(const ds_dbw_msgs::msg::VehicleVelocity::SharedPtr msg) {
    // reading messages about UGV current velocity

    float vel = msg->vehicle_velocity_brake; // m/s, measured by brakes (wheel sensors)
  }


  void system_report_callback(const ds_dbw_msgs::msg::SystemReport::SharedPtr msg) {
    // reading messages about UGV internal system

    bool ugv_blocked = msg->inhibit;
    int system_state = msg->state.value; // MANUAL=0, READY=1, ACTIVE=2, FAULT=7
    int reason_disengage = msg->reason_disengage;
    bool overrided = msg->override; // Any steer/brake/throttle/gear override
    bool ready = msg->ready; // All steer/brake/throttle ready, and gear not faulted
    bool enabled = msg->enabled; // All steer/brake/throttle enabled, and gear not faulted (or any steer/brake/throttle enabled for Mode>=AllOrNone)
    bool fault = msg->fault; // Fault in any steer/brake/throttle/gear

  }

  void remote_report_callback(const ds_dbw_msgs::msg::RemoteReport::SharedPtr msg) {
    // reading messages about UGV remote system telemetry
  }

  void drive_mode_report_callback(const ds_dbw_msgs::msg::DriveModeReport::SharedPtr msg) {
    // reading messages about UGV drive mode
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // reading messages about UGV GPS location
  }

//   void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
//     auto manual = mavros_msgs::msg::ManualControl();

//     manual.x = static_cast<int16_t>(msg->axes[0] * 1000);  // Roll
//     manual.y = static_cast<int16_t>(msg->axes[1] * 1000);  // Pitch
//     manual.z = static_cast<int16_t>(msg->axes[2] * 1000);  // Throttle
//     manual.r = static_cast<int16_t>(msg->axes[3] * 1000);  // Yaw

//     uint16_t buttons = 0;
//     for (size_t i = 0; i < msg->buttons.size(); ++i) {
//       if (msg->buttons[i]) {
//         buttons |= (1 << i);
//       }
//     }
//     manual.buttons = buttons;
//     //manual.target = 1;

//     manual_pub_->publish(manual);

//     mavlink_message_t mav_msg;

//     mavlink_msg_manual_control_pack(
//     123,
//     123,
//     &mav_msg,
//     0,
//     manual.x,
//     manual.y,
//     manual.z,
//     manual.r,
//     manual.buttons,
//     0,
//     0,
//     0,
//     0,
//     0,
//     0,
//     0,
//     0,
//     0,
//     0);

//     uint8_t buffer[360];
//     int len = mavlink_msg_to_send_buffer(buffer, &mav_msg);
//     RCLCPP_INFO(this->get_logger(), "The LEN is: %d", len);

//     int bytes_sent = sendto(sock_, buffer, len, 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
//     if (bytes_sent < 0) {
//         RCLCPP_INFO(this->get_logger(), "UDP send failed");
//     } else {
//         RCLCPP_INFO(this->get_logger(), "UDP send OK");
//     }

//   }

  int sock_;
  struct sockaddr_in remote_addr_{};
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<ds_dbw_msgs::msg::SteeringReport>::SharedPtr steering_sub_;
  rclcpp::Subscription<ds_dbw_msgs::msg::GearReport>::SharedPtr gear_sub_;
  rclcpp::Subscription<ds_dbw_msgs::msg::FuelLevel>::SharedPtr fuel_sub_; // mavlink: SYS_STATUS?
  rclcpp::Subscription<ds_dbw_msgs::msg::VehicleVelocity>::SharedPtr velocity_sub_; // mavlink: VFR_HUD
  rclcpp::Subscription<ds_dbw_msgs::msg::SystemReport>::SharedPtr sys_report_sub_; // mavlink: SYS_STATUS? | HEARTBEAT
  rclcpp::Subscription<ds_dbw_msgs::msg::RemoteReport>::SharedPtr rem_report_sub_; // mavlink: SYS_STATUS?
  rclcpp::Subscription<ds_dbw_msgs::msg::DriveModeReport>::SharedPtr drive_mode_report_sub_; // mavlink: STATUSTEXT
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_; // mavlink: GLOBAL_POSITION_INT
  
  //rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;

};
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UGVTelemetry>());
  rclcpp::shutdown();
  return 0;
}