#include <mavlink/v2.0/common/mavlink.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "ds_dbw_msgs/msg/gear_report.hpp"
#include "ds_dbw_msgs/msg/steering_report.hpp"
#include "ds_dbw_msgs/msg/fuel_level.hpp"
#include "ds_dbw_msgs/msg/vehicle_velocity.hpp"
#include "ds_dbw_msgs/msg/system_report.hpp"
#include "ds_dbw_msgs/msg/remote_report.hpp"
#include "ds_dbw_msgs/msg/drive_mode_report.hpp"
#include "ds_dbw_msgs/msg/throttle_info.hpp"
#include "ds_dbw_msgs/msg/brake_report.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

struct UVG_telemetry_struct{
  uint16_t gear_curr = 0; // mavlink Target = 1
  uint16_t gear_cmd = 0; // mavlink Target = 1
  uint8_t gear_reject = 0; // mavlink Target = 1
  int16_t gear_ready = 0; // mavlink Target = 1
  int16_t gear_fault = 0; // mavlink Target = 1
  int16_t steer_steering_wheel_angle = 0; // mavlink Target = 1
  int16_t steer_enabled = 0; // mavlink Target = 1
  int16_t steer_ready = 0 ; // mavlink Target = 1
  int16_t steer_fault = 0; // mavlink Target = 1
  int16_t fuel_level = 0; // mavlink Target = 1
  int16_t fuel_range = 0; // mavlink Target = 1
  int16_t fuel_odometer = 0; // mavlink Target = 1
  int16_t system_state = 0; // mavlink Target = 1
  int16_t system_enabled = 0; // mavlink Target = 1
  int16_t system_fault = 0; // mavlink Target = 1
  int16_t system_inhibit = 0; // mavlink Target = 2
  int16_t system_reason_not_ready = 0; // mavlink Target = 2
  int16_t system_ready = 0; // mavlink Target = 2
  uint16_t gps_latitude = 0; // mavlink Target = 2
  uint16_t gps_longitude = 0; // mavlink Target = 2
  int16_t gps_altitude = 0; // mavlink Target = 2
  int16_t velocity_vehicle_velocity_propulsion = 0; // mavlink Target = 2
  int16_t velocity_vehicle_velocity_brake = 0; // mavlink Target = 2
  int16_t accel_pedal_pc = 0; // mavlink Target = 2
  int16_t accel_engine_rpm = 0; // mavlink Target = 2
  int16_t accel_engine_throttle_valve_pc = 0; // mavlink Target = 2
  uint8_t brake_enabled = 0; // mavlink Target = 2
  int16_t brake_torque_pedal = 0; // mavlink Target = 2
  int16_t brake_fault = 0; // mavlink Target = 2
};

struct MessagesFrequency_struct{ // Frequencies of subscribded messages in Hz
  int16_t gear = 0;
  int16_t steering = 0;
  int16_t fuel_level = 0;
  int16_t system = 0;
  int16_t gps = 0;
  int16_t velocity = 0;
  int16_t throttle = 0;
  int16_t brake = 0;
};

class MsgFrequency {
  public:
    MsgFrequency(std::string name) {
      name_ = name;
    }

    void Tick(std::shared_ptr<rclcpp::Node> node) { // counts messages and calculates frequency
      auto now = node->get_clock()->now();

      if (msg_count_ > 0) {
        auto dt = now.seconds() - last_msg_time_;
        hz_ = 1.0 / dt;
      }

      last_msg_time_ = now.seconds();
      msg_count_++;
    }

    double GetFreq(std::shared_ptr<rclcpp::Node> node) { // returns current frequency
      auto now = node->get_clock()->now();
      auto dt = now.seconds() - last_msg_time_;
      //std::cout << "dt: " <<  name_ << " = " << dt << std::endl;
      if (dt > 2.0) {
        hz_ = 0.0;
        //std::cout << "Reset freq for " <<  name_ << std::endl;
      }

      return hz_;
    }


  private:
    size_t msg_count_ = 0;
    double last_msg_time_ = 0.0;
    double hz_ = 0;
    std::string name_;

};

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
      "/vehicle/steering/report", 1,
      std::bind(&UGVTelemetry::steering_report_callback, this, std::placeholders::_1));

    // subscribe Gear info
    gear_sub_ = this->create_subscription<ds_dbw_msgs::msg::GearReport>(
      "/vehicle/gear/report", 1,
      std::bind(&UGVTelemetry::gear_report_callback, this, std::placeholders::_1));

    // subscribe Throttle info
    thottle_sub_ = this->create_subscription<ds_dbw_msgs::msg::ThrottleInfo>(
      "/vehicle/throttle/info", 1,
      std::bind(&UGVTelemetry::throttle_report_callback, this, std::placeholders::_1));

    // subscribe Brake info
    brake_sub_ = this->create_subscription<ds_dbw_msgs::msg::BrakeReport>(
      "/vehicle/brake/report", 1,
      std::bind(&UGVTelemetry::brake_report_callback, this, std::placeholders::_1));

    // manual_pub_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
    //   "/mavros/manual_control/send", 10
    // );
    ugv_state_ = std::make_shared<UVG_telemetry_struct>();
    ugv_telemetry_freq_ = std::make_shared<MessagesFrequency_struct>();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&UGVTelemetry::check_subs_, this));

    timer2_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UGVTelemetry::send_data_to_gc, this));

    timer3_ = this->create_wall_timer(std::chrono::milliseconds(110), std::bind(&UGVTelemetry::refresh_data, this));


    gear_hz_ = std::make_shared<MsgFrequency>("gearHz");
    steering_hz_ = std::make_shared<MsgFrequency>("steerHz");
    fuel_level_hz_ = std::make_shared<MsgFrequency>("fuelHz");
    system_hz_ = std::make_shared<MsgFrequency>("systemHz");
    gps_hz_ = std::make_shared<MsgFrequency>("gpsHz");
    velocity_hz_ = std::make_shared<MsgFrequency>("velocityHz");
    throttle_hz_ = std::make_shared<MsgFrequency>("throttleHz");
    brake_hz_ = std::make_shared<MsgFrequency>("brakeHz");

    RCLCPP_INFO(this->get_logger(), "MAVROS UGVTelemetry node started.");

    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
        RCLCPP_INFO(this->get_logger(), "Socket creation failed.");
    }

    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(14551);  // Target port (e.g. QGroundControl or autopilot)
    remote_addr_.sin_addr.s_addr = inet_addr("172.25.64.196");  // Target IP / local IP = 172.25.65.12, Remote IP = 172.25.64.197
  }

private:

  void refresh_data() {
    // checks freq timeout and resets values

    fuel_level_hz_->GetFreq(this->shared_from_this());
    throttle_hz_->GetFreq(this->shared_from_this());
    gear_hz_->GetFreq(this->shared_from_this());
    steering_hz_->GetFreq(this->shared_from_this());
    velocity_hz_->GetFreq(this->shared_from_this());
    system_hz_->GetFreq(this->shared_from_this());
    gps_hz_->GetFreq(this->shared_from_this());
    brake_hz_->GetFreq(this->shared_from_this());

  }

  void brake_report_callback(const ds_dbw_msgs::msg::BrakeReport::SharedPtr msg) {
    // reading messages from UGV about the brakes

    brake_hz_->Tick(this->shared_from_this());
    ugv_telemetry_freq_->brake = static_cast<uint16_t>(brake_hz_->GetFreq(this->shared_from_this()) * 10.0);

    ugv_state_->brake_enabled = static_cast<uint8_t>(msg->enabled);
    ugv_state_->brake_torque_pedal = static_cast<int16_t>(msg->torque_input);
    ugv_state_->brake_fault = static_cast<int16_t>(msg->fault);
  }

  void fuel_callback(const ds_dbw_msgs::msg::FuelLevel::SharedPtr msg) {
    // reading messages from UGV about the fuel level

    fuel_level_hz_->Tick(this->shared_from_this());
    ugv_telemetry_freq_->fuel_level = static_cast<uint16_t>(fuel_level_hz_->GetFreq(this->shared_from_this()) * 10.0);

    ugv_state_->fuel_level = static_cast<int16_t>(msg->fuel_level);
    ugv_state_->fuel_range = static_cast<int16_t>(msg->fuel_range);
    ugv_state_->fuel_odometer = static_cast<int16_t>(msg->odometer);
  }

  void send_data_to_gc() {
    // sends telemetry data to the GroundControl using mavlink messages


    // Fill data for Target = 1
    mavlink_message_t mav_msg;

    mavlink_msg_manual_control_pack(
    123, // system_id, uint8_t
    1, // component_id, uint8_t
    &mav_msg, // &mav_msg, mavlink_message_t*
    1, // target, uint8_t
    ugv_state_->steer_steering_wheel_angle, // x, int16_t
    ugv_state_->steer_enabled, // y, int16_t
    ugv_state_->steer_ready, // z, int16_t
    ugv_state_->steer_fault, // r, int16_t
    ugv_state_->gear_curr, // buttons, uint16_t
    ugv_state_->gear_cmd, // buttons2, uint16_t
    ugv_state_->gear_reject, // enabled_extensions, uint8_t
    ugv_state_->gear_ready, // s, int16_t
    ugv_state_->gear_fault, // t, int16_t
    ugv_state_->fuel_level, // aux1, int16_t
    ugv_state_->fuel_range, // aux2, int16_t
    ugv_state_->fuel_odometer, // aux3, int16_t
    ugv_state_->system_state, // aux4, int16_t
    ugv_state_->system_enabled, // aux5, int16_t
    ugv_state_->system_fault // aux6, int16_t
    );

    uint8_t buffer[360];
    int len = mavlink_msg_to_send_buffer(buffer, &mav_msg);
    //RCLCPP_INFO(this->get_logger(), "The LEN is: %d", len);

    int bytes_sent = sendto(sock_, buffer, len, 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    if (bytes_sent < 0) {
        RCLCPP_INFO(this->get_logger(), "UDP send failed: mavlink target=1, manual_control");
    } else {
        
    }

    // Fill data for Target = 2
    mavlink_message_t mav_msg2;

    mavlink_msg_manual_control_pack(
    123, // system_id, uint8_t
    1, // component_id, uint8_t
    &mav_msg2, // &mav_msg, mavlink_message_t*
    2, // target, uint8_t
    ugv_state_->system_inhibit, // x, int16_t
    ugv_state_->system_reason_not_ready, // y, int16_t
    ugv_state_->system_ready, // z, int16_t
    ugv_state_->gps_altitude, // r, int16_t
    ugv_state_->gps_latitude, // buttons, uint16_t
    ugv_state_->gps_longitude, // buttons2, uint16_t
    ugv_state_->brake_enabled, // enabled_extensions, uint8_t
    ugv_state_->brake_torque_pedal, // s, int16_t
    ugv_state_->brake_fault, // t, int16_t
    ugv_state_->velocity_vehicle_velocity_propulsion, // aux1, int16_t
    ugv_state_->velocity_vehicle_velocity_brake, // aux2, int16_t
    ugv_state_->accel_pedal_pc, // aux3, int16_t
    ugv_state_->accel_engine_rpm, // aux4, int16_t
    ugv_state_->accel_engine_throttle_valve_pc, // aux5, int16_t
    0 // aux6, int16_t
    );

    uint8_t buffer2[360];
    int len2 = mavlink_msg_to_send_buffer(buffer2, &mav_msg2);
    //RCLCPP_INFO(this->get_logger(), "The LEN is: %d", len);

    int bytes_sent2 = sendto(sock_, buffer2, len2, 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    if (bytes_sent2 < 0) {
        RCLCPP_INFO(this->get_logger(), "UDP send failed: mavlink target=2, manual_control");
    } else {
        
    }

    // Fill data for Target = 3
    mavlink_message_t mav_msg3;

    mavlink_msg_manual_control_pack(
    123, // system_id, uint8_t
    1, // component_id, uint8_t
    &mav_msg3, // &mav_msg, mavlink_message_t*
    3, // target, uint8_t
    //ugv_telemetry_freq_->gear, // x, int16_t
    static_cast<uint16_t>(gear_hz_->GetFreq(this->shared_from_this()) * 10.0),
    //ugv_telemetry_freq_->steering, // y, int16_t
    static_cast<uint16_t>(steering_hz_->GetFreq(this->shared_from_this()) * 10.0),
    //ugv_telemetry_freq_->fuel_level, //
    static_cast<uint16_t>(fuel_level_hz_->GetFreq(this->shared_from_this()) * 10.0), // z, int16_t
    //ugv_telemetry_freq_->system, // r, int16_t
    static_cast<uint16_t>(system_hz_->GetFreq(this->shared_from_this()) * 10.0),
    0, // buttons, uint16_t
    0, // buttons2, uint16_t
    0, // enabled_extensions, uint8_t
    //ugv_telemetry_freq_->gps, // s, int16_t
    static_cast<uint16_t>(gps_hz_->GetFreq(this->shared_from_this()) * 10.0),
    //ugv_telemetry_freq_->velocity, // t, int16_t
    static_cast<uint16_t>(velocity_hz_->GetFreq(this->shared_from_this()) * 10.0),
    //ugv_telemetry_freq_->throttle, // aux1, int16_t
    static_cast<uint16_t>(throttle_hz_->GetFreq(this->shared_from_this()) * 10.0),
    static_cast<uint16_t>(brake_hz_->GetFreq(this->shared_from_this()) * 10.0), // aux2, int16_t
    0, // aux3, int16_t
    0, // aux4, int16_t
    0, // aux5, int16_t
    0 // aux6, int16_t
    );

    

    uint8_t buffer3[360];
    int len3 = mavlink_msg_to_send_buffer(buffer3, &mav_msg3);
    //RCLCPP_INFO(this->get_logger(), "The LEN is: %d", len);

    int bytes_sent3 = sendto(sock_, buffer3, len3, 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    if (bytes_sent3 < 0) {
        RCLCPP_INFO(this->get_logger(), "UDP send failed: mavlink target=3, manual_control");
    } else {

    }

  }

  void check_subs_() {
    // checks subscribed topics are still arriving

    if (steering_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/steering/report' msgs published ('%s')?", this->get_name());
    }

    if (gear_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/gear/report' msgs published ('%s')?", this->get_name());
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

    if (thottle_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/throttle/info' msgs published ('%s')?", this->get_name());
    }

    if (brake_sub_->get_publisher_count() == 0) {
      RCLCPP_INFO(this->get_logger(), "Are '/vehicle/brake/report' msgs published ('%s')?", this->get_name());
    } 
    
  }

  void throttle_report_callback(const ds_dbw_msgs::msg::ThrottleInfo::SharedPtr msg) {
    // reading throttle info messages from UGV

    throttle_hz_->Tick(this->shared_from_this());
    
    ugv_telemetry_freq_->throttle = static_cast<uint16_t>(throttle_hz_->GetFreq(this->shared_from_this()) * 10.0);

    ugv_state_->accel_pedal_pc = static_cast<int16_t>(msg->accel_pedal_pc);
    ugv_state_->accel_engine_rpm = static_cast<int16_t>(msg->engine_rpm);
    ugv_state_->accel_engine_throttle_valve_pc = static_cast<int16_t>(msg->engine_throttle_valve_pc);
  }

  void gear_report_callback(const ds_dbw_msgs::msg::GearReport::SharedPtr msg) {
  // reading messages from UGV about gear
    gear_hz_->Tick(this->shared_from_this());

    ugv_telemetry_freq_->gear = static_cast<uint16_t>(gear_hz_->GetFreq(this->shared_from_this()) * 10.0);

    ugv_state_->gear_curr = static_cast<uint16_t>(msg->gear.value);
    ugv_state_->gear_cmd = static_cast<uint16_t>(msg->cmd.value);
    ugv_state_->gear_reject = static_cast<uint8_t>(msg->reject.value);
    ugv_state_->gear_ready = static_cast<int16_t>(msg->ready);
    ugv_state_->gear_fault = static_cast<int16_t>(msg->fault);

  }

  void steering_report_callback(const ds_dbw_msgs::msg::SteeringReport::SharedPtr msg) {
  // reading messages from UGV about steering

    steering_hz_->Tick(this->shared_from_this());
    ugv_telemetry_freq_->steering = static_cast<uint16_t>(steering_hz_->GetFreq(this->shared_from_this()) * 10.0);

    ugv_state_->steer_steering_wheel_angle = static_cast<int16_t>(msg->steering_wheel_angle);
    ugv_state_->steer_enabled = static_cast<int16_t>(msg->enabled);
    ugv_state_->steer_ready = static_cast<int16_t>(msg->ready);
    ugv_state_->steer_fault = static_cast<int16_t>(msg->fault);
  }

  void velocity_callback(const ds_dbw_msgs::msg::VehicleVelocity::SharedPtr msg) {
    // reading messages about UGV current velocity

    velocity_hz_->Tick(this->shared_from_this());
    ugv_telemetry_freq_->velocity = static_cast<uint16_t>(velocity_hz_->GetFreq(this->shared_from_this()) * 10.0);

    ugv_state_->velocity_vehicle_velocity_propulsion = static_cast<int16_t>(msg->vehicle_velocity_propulsion);
    ugv_state_->velocity_vehicle_velocity_brake = static_cast<int16_t>(msg->vehicle_velocity_brake);
  }


  void system_report_callback(const ds_dbw_msgs::msg::SystemReport::SharedPtr msg) {
    // reading messages about UGV internal system

    system_hz_->Tick(this->shared_from_this());
    ugv_telemetry_freq_->system = static_cast<uint16_t>(system_hz_->GetFreq(this->shared_from_this()) * 10.0);
    
    ugv_state_->system_state = static_cast<int16_t>(msg->state.value);
    ugv_state_->system_enabled = static_cast<int16_t>(msg->enabled);
    ugv_state_->system_fault = static_cast<int16_t>(msg->fault);
    ugv_state_->system_inhibit = static_cast<int16_t>(msg->inhibit);
    ugv_state_->system_reason_not_ready = static_cast<int16_t>(msg->reason_not_ready);
    ugv_state_->system_ready = static_cast<int16_t>(msg->ready);

  }

  void remote_report_callback(const ds_dbw_msgs::msg::RemoteReport::SharedPtr msg) {
    // reading messages about UGV remote system telemetry
  }

  void drive_mode_report_callback(const ds_dbw_msgs::msg::DriveModeReport::SharedPtr msg) {
    // reading messages about UGV drive mode
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // reading messages about UGV GPS location

    gps_hz_->Tick(this->shared_from_this());
    ugv_telemetry_freq_->gps = static_cast<uint16_t>(gps_hz_->GetFreq(this->shared_from_this()) * 10.0);

    ugv_state_->gps_latitude = static_cast<uint16_t>(msg->latitude);
    ugv_state_->gps_longitude = static_cast<uint16_t>(msg->longitude);
    ugv_state_->gps_altitude = static_cast<int16_t>(msg->altitude);
  }

  int sock_;
  struct sockaddr_in remote_addr_{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::TimerBase::SharedPtr timer3_;
  std::shared_ptr<UVG_telemetry_struct> ugv_state_;
  std::shared_ptr<MessagesFrequency_struct> ugv_telemetry_freq_;

  rclcpp::Subscription<ds_dbw_msgs::msg::SteeringReport>::SharedPtr steering_sub_;
  rclcpp::Subscription<ds_dbw_msgs::msg::GearReport>::SharedPtr gear_sub_;
  rclcpp::Subscription<ds_dbw_msgs::msg::FuelLevel>::SharedPtr fuel_sub_;
  rclcpp::Subscription<ds_dbw_msgs::msg::VehicleVelocity>::SharedPtr velocity_sub_; 
  rclcpp::Subscription<ds_dbw_msgs::msg::SystemReport>::SharedPtr sys_report_sub_; 
  rclcpp::Subscription<ds_dbw_msgs::msg::RemoteReport>::SharedPtr rem_report_sub_; 
  rclcpp::Subscription<ds_dbw_msgs::msg::DriveModeReport>::SharedPtr drive_mode_report_sub_; 
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<ds_dbw_msgs::msg::ThrottleInfo>::SharedPtr thottle_sub_;
  rclcpp::Subscription<ds_dbw_msgs::msg::BrakeReport>::SharedPtr brake_sub_;
  
  
  std::shared_ptr<MsgFrequency> gear_hz_;
  std::shared_ptr<MsgFrequency> steering_hz_;
  std::shared_ptr<MsgFrequency> fuel_level_hz_;
  std::shared_ptr<MsgFrequency> system_hz_;
  std::shared_ptr<MsgFrequency> gps_hz_;
  std::shared_ptr<MsgFrequency> velocity_hz_;
  std::shared_ptr<MsgFrequency> throttle_hz_;
  std::shared_ptr<MsgFrequency> brake_hz_;

};
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UGVTelemetry>());
  rclcpp::shutdown();
  return 0;
}