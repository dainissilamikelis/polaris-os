#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/mavlink.hpp"
#include <mavlink/v2.0/common/mavlink.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

class JoyToManual : public rclcpp::Node {
public:
  JoyToManual() : Node("joy_to_manual") {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&JoyToManual::joy_callback, this, std::placeholders::_1)
    );

    manual_pub_ = this->create_publisher<mavros_msgs::msg::ManualControl>(
      "/mavros/manual_control/send", 10
    );

    RCLCPP_INFO(this->get_logger(), "Joystick to MAVROS ManualControl node started.");

    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
        RCLCPP_INFO(this->get_logger(), "Socket creation failed.");
    }

    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(14550);  // Target port (e.g. QGroundControl or autopilot)
    remote_addr_.sin_addr.s_addr = inet_addr("172.25.65.12");  // Target IP / local IP = 172.25.65.12, Remote IP = 172.25.64.197
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

    // calculate time difference between the current and the prev msg
    auto sec = msg->header.stamp.sec;
    auto nsec = msg->header.stamp.nanosec;
    double curr_time_in_miliseconds = (sec + nsec * 1e-9) * 1000;
    double time_diff_milisec = curr_time_in_miliseconds - prev_time_in_miliseconds_;
    prev_time_in_miliseconds_ = curr_time_in_miliseconds;

    
    auto manual = mavros_msgs::msg::ManualControl();

    manual.x = static_cast<int16_t>(msg->axes[3] * 1000);  // Steering
    manual.y = static_cast<int16_t>(msg->axes[5] * 1000);  // Throttle
    manual.z = static_cast<int16_t>(msg->axes[2] * 1000);  // Break
    manual.r = static_cast<int16_t>(msg->axes[0] * 1000);  // Steering [ALT]

    uint16_t buttons = 0;
    for (size_t i = 0; i < msg->buttons.size(); ++i) {
      if (msg->buttons[i]) {
        buttons |= (1 << i);
      }
    }

    manual.aux1 = static_cast<int16_t>(msg->axes[6] * 1000);  // TURN LEFT | TURN RIGHT | AXIS_STEER_MULT_2
    manual.aux2 = static_cast<int16_t>(msg->axes[7] * 1000);  // AXIS_BRAKE_PRECHARGET | AXIS_STEER_MULT_1
    manual.aux3 = static_cast<int16_t>(msg->axes[1] * 1000);  // RESERVED
    manual.aux4 = static_cast<int16_t>(msg->axes[4] * 1000);  // RESERVED
    
    manual.buttons = buttons;
    //manual.target = 1;

    //manual_pub_->publish(manual); // ??

    mavlink_message_t mav_msg;

    mavlink_msg_manual_control_pack(
    123,
    123,
    &mav_msg,
    0,
    manual.x,
    manual.y,
    manual.z,
    manual.r,
    manual.buttons,
    0, 
    0, 
    0, 
    0, 
    manual.aux1, 
    manual.aux2, 
    manual.aux3, 
    manual.aux4, 
    0,
    time_diff_milisec);

    uint8_t buffer[360];
    int len = mavlink_msg_to_send_buffer(buffer, &mav_msg);
    //RCLCPP_INFO(this->get_logger(), "The LEN is: %d", len);

    int bytes_sent = sendto(sock_, buffer, len, 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    if (bytes_sent < 0) {
        RCLCPP_INFO(this->get_logger(), "UDP send failed");
    } else {
        //RCLCPP_INFO(this->get_logger(), "UDP send OK");
    }

  }

  int sock_;
  double prev_time_in_miliseconds_ = 0.0;
  struct sockaddr_in remote_addr_{};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;

};
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToManual>());
  rclcpp::shutdown();
  return 0;
}