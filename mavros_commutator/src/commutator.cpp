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
#include <bitset>
#include <map>

enum class JoyHoldButtons { // MAPPING struct
    LOW_GEAR = 1 << 0, // A button
    REVERSE_GEAR = 1 << 1, // B button
    PARKING_GEAR = 1 << 2, // X button
    HIGH_GEAR = 1 << 3, // Y button
    NEUTRAL_GEAR = 1 << 5, // Top-right front button
    WD_MODE = 1 << 8, // 'Logitech' central button
    ENABLE = 1 << 7, // Start
    LED = 1 << 4, // LEDs not holded
    WIPER = 1 << 6 // WIPER not holded 
};



class ButtonMaskProcessor {
    public:
        ButtonMaskProcessor() {

        }

        bool isPressed(JoyHoldButtons button, uint16_t state) {
            return (state & static_cast<uint16_t>(button));
        }
};



class ButtonValue {
    // Class for ON/OFF button value storage
    public:
        ButtonValue() {
            
            pressed_ = false;
            value_ = false;
        }

        void SetValue (bool value){
        // Set new value of a button
            value_ = value;
        }

        uint16_t GetValue() {
        // Returns the current value of a button
            return value_;
        }

        void Reset() {
        // Set IsOncePressed = false and the current value = false
            pressed_ = false;
            value_ = false;
        }

        bool IsOncePressed() {
        // Returns the current IsOncePressed status
            return pressed_;
        }

        void SetPressed() {
        // Set IsOncePressed = tru if a button holding time was reached
            pressed_ = true;
            value_ = false;

        }

        rclcpp::Time current_time;
    private:
        bool value_;
        bool pressed_;
        

};

class ButtonPressed : public rclcpp::Node {
// The class handles joystick button press events, provided a predefined time delay condition is met.
    public:
        explicit ButtonPressed(const std::string & node_name, std::shared_ptr<ButtonValue> button_value) : rclcpp::Node(node_name) {
            RCLCPP_INFO(this->get_logger(), "ButtonPressedNode '%s' node started.", this->get_name());

            button_value_ = button_value;
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ButtonPressed::process_value, this));
            prev_value_ = 0;
        }

        void SetHoldTime(double secs) {
        // Set how long to wait in seconds and then a button marked as pressed
            hold_time_ = secs;
        }
    
    private:
        std::shared_ptr<ButtonValue> button_value_;
        rclcpp::TimerBase::SharedPtr timer_;
        uint16_t prev_value_;
        rclcpp::Time prev_time_ = this->now();
        double hold_time_ = 2.0; // secs

        void process_value() {
            bool curr_value = button_value_->GetValue();
            if (prev_value_ != curr_value) {
                prev_time_ =  this->now(); // reset time
                prev_value_ = curr_value;
            }
            
            if (!button_value_->IsOncePressed()) { // a button is not enabled
                if (prev_value_ == 1) { // a button is still pressed
                    rclcpp::Duration time_diff = this->now() - prev_time_;
                    double seconds = time_diff.seconds();

                    if (seconds >= hold_time_) {
                        button_value_->SetPressed();
                    }
                }
            }
        }
    
};

class TAK_telemetry : public rclcpp::Node {
// The class communicates mavlink messages to the TAK
    public:
        TAK_telemetry() : rclcpp::Node("tak_telemetry_node") {
            RCLCPP_INFO(this->get_logger(), "ButtonPressedNode '%s' node started.", this->get_name());

            sock_ = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock_ < 0) {
                RCLCPP_INFO(this->get_logger(), "Socket creation failed.");
            }

            remote_addr_.sin_family = AF_INET;
            remote_addr_.sin_port = htons(14552);  // Target port (e.g. QGroundControl or autopilot)
            remote_addr_.sin_addr.s_addr = inet_addr("172.25.116.22");  // Target IP

            if (bind(sock_, (const struct sockaddr*)&remote_addr_, sizeof(remote_addr_)) < 0) {
                RCLCPP_INFO(this->get_logger(), "Socket bind failed.");
                close(sock_);
            }

            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TAK_telemetry::send_heartbit_msg, this));

            timer2_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TAK_telemetry::send_statustext_msg, this));

            timer3_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TAK_telemetry::send_sys_status_msg, this));

            timer4_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TAK_telemetry::send_global_position_int_msg, this));

            timer5_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TAK_telemetry::send_vfr_hud_msg, this));
        }

        ~TAK_telemetry() {
            close(sock_);
        }
    
    private:
        int sock_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;
        rclcpp::TimerBase::SharedPtr timer3_;
        rclcpp::TimerBase::SharedPtr timer4_;
        rclcpp::TimerBase::SharedPtr timer5_;

        uint16_t prev_value_;
        rclcpp::Time prev_time_ = this->now();
        double hold_time_ = 2.0; // secs
        struct 
            sockaddr_in remote_addr_{};

        void send_heartbit_msg() {
            // create and send HEARTBIT message to the TAK

            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_msg_heartbeat_pack(
            1,     // system ID
            1,   // component ID
            &msg,
            MAV_TYPE_GROUND_ROVER,
            MAV_AUTOPILOT_GENERIC,
            MAV_MODE_MANUAL_DISARMED,
            0,     // custom mode
            MAV_STATE_ACTIVE
            );

            int len = mavlink_msg_to_send_buffer(buf, &msg);
            RCLCPP_INFO(this->get_logger(), "The LEN is: %d", len);

            int bytes_sent = sendto(sock_, buf, len, 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
            if (bytes_sent < 0) {
                RCLCPP_INFO(this->get_logger(), "heartbit UDP send failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "heartbit UDP send OK");
            }
        }

        void send_statustext_msg() {
            // create and send STATUSTEXT message to the TAK

            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];

            const char* status = "UGV is ready";

            mavlink_msg_statustext_pack(
            1,     // system ID
            1,   // component ID
            &msg,
            MAV_SEVERITY_INFO,
            status,
            0, // Unique ID for message grouping (extension)
            0 // Sequence number for chunked messages (extension)
            );

            int len = mavlink_msg_to_send_buffer(buf, &msg);
            RCLCPP_INFO(this->get_logger(), "The LEN is: %d", len);

            int bytes_sent = sendto(sock_, buf, len, 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
            if (bytes_sent < 0) {
                RCLCPP_INFO(this->get_logger(), "statustext UDP send failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "statustext UDP send OK");
            }

        }

        void send_sys_status_msg() {
            // create and send SYS_STATUS message to the TAK

            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];

            uint32_t sensors_present = MAV_SYS_STATUS_SENSOR_3D_GYRO |
                               MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                               MAV_SYS_STATUS_SENSOR_GPS;
            
            uint32_t sensors_enabled = sensors_present;
            uint32_t sensors_health  = sensors_present;
            uint16_t load = 0;               // 0% CPU load
            uint16_t voltage_battery = 0;  // 0.0 V, used as FUEL
            int16_t current_battery = 0;     // 0.0 A, used as FUEL
            int8_t battery_remaining = 77;     // 100% FUEL level of the UGV
            uint16_t drop_rate_comm = 0;      // 0% drop rate
            uint16_t errors_comm = 0;
            uint16_t errors_count1 = 0;
            uint16_t errors_count2 = 0;
            uint16_t errors_count3 = 0;
            uint16_t errors_count4 = 0;
            uint32_t battery_current_consumed = 0;  // NEW
            uint32_t battery_energy_consumed = 0;   // NEW
            uint32_t communication_errors = 0;       // NEW

            mavlink_msg_sys_status_pack(
                1,  // system ID
                1,  // component ID
                &msg,
                sensors_present,
                sensors_enabled,
                sensors_health,
                load,
                voltage_battery,
                current_battery,
                battery_remaining,
                drop_rate_comm,
                errors_comm,
                errors_count1,
                errors_count2,
                errors_count3,
                errors_count4,
                battery_current_consumed,
                battery_energy_consumed,
                communication_errors
            );

            int len = mavlink_msg_to_send_buffer(buf, &msg);
            RCLCPP_INFO(this->get_logger(), "The LEN is: %d", len);

            int bytes_sent = sendto(sock_, buf, len, 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
            if (bytes_sent < 0) {
                RCLCPP_INFO(this->get_logger(), "sys_status UDP send failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "sys_status UDP send OK");
            }

        }
        
        void send_global_position_int_msg() {
            // create and send GLOBAL_POSITION_INT message to the TAK

            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];

            mavlink_msg_global_position_int_pack(
            1,     // system ID
            1,   // component ID
            &msg,
            123456,        // time_boot_ms
            569521350,     // lat (56.952117°) 56.952135, 24.078829
            240788290,     // lon (24.079051°)
            12000,         // alt (12.0 m AMSL)
            500,           // relative_alt (0.5 m above ground)
            100,           // vx (1.0 m/s North)
            0,             // vy (0 m/s East)
            -50,           // vz (-0.5 m/s Down)
            9000           // hdg (90.00° East)
            );

            int len = mavlink_msg_to_send_buffer(buf, &msg);
            RCLCPP_INFO(this->get_logger(), "The LEN is: %d", len);

            int bytes_sent = sendto(sock_, buf, len, 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
            if (bytes_sent < 0) {
                RCLCPP_INFO(this->get_logger(), "GLOBAL_POSITION_INT UDP send failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "GLOBAL_POSITION_INT UDP send OK");
            }

        }

        void send_vfr_hud_msg() {
            // create and send VFR_HUD message to the TAK

            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];

            mavlink_msg_vfr_hud_pack(
                1,    // system ID
                1,  // component ID
                &msg,
                12.5,     // airspeed (m/s)
                13.0,     // groundspeed (m/s)
                90,       // heading (degrees)
                75,       // throttle (%)
                120.0,    // altitude (meters)
                1.5       // climb rate (m/s)
            );

            int len = mavlink_msg_to_send_buffer(buf, &msg);
            RCLCPP_INFO(this->get_logger(), "The LEN is: %d", len);

            int bytes_sent = sendto(sock_, buf, len, 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
            if (bytes_sent < 0) {
                RCLCPP_INFO(this->get_logger(), "VFR_HUD UDP send failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "VFR_HUD UDP send OK");
            }

        }


};

class JoyReader : public rclcpp::Node {
    public:
    JoyReader(std::shared_ptr<std::map<JoyHoldButtons, std::shared_ptr<ButtonValue>>> oncePressButtons) : Node("joy_reader") {
        // The Node listening maviklink MANUAL_CONTROL messages, then publishes appropriate sensor_msgs::msg::Joy messages under /joy_polaris ROS topic.

        RCLCPP_INFO(this->get_logger(), "Joystick commands reader node started.");

        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_INFO(this->get_logger(), "Socket creation failed.");
        }

        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(14550);  // Target port (e.g. QGroundControl or autopilot)
        server_addr_.sin_addr.s_addr = INADDR_ANY;  // Target IP

        if (bind(sock_, (const struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
            RCLCPP_INFO(this->get_logger(), "Socket bind failed.");
            close(sock_);
        }

        oncePressButtons_ = oncePressButtons;
        using namespace std::chrono_literals;
        prev_time_ = now();
        
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&JoyReader::receive_loop, this));
        joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy_polaris", 1);
        //timer_heartbit_mavlink_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&JoyReader::receive_loop, this));
    }

    ~JoyReader() {
        close(sock_);
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }

    private:
        void receive_loop() {
            char buffer[1024];
            sockaddr_in sender{};
            socklen_t sender_len = sizeof(sender);

            ButtonMaskProcessor buttonProcessor;

            while (rclcpp::ok()) {
                ssize_t len = recvfrom(sock_, buffer, sizeof(buffer) - 1, 0,
                                    (struct sockaddr*)&sender, &sender_len);
                // if (len > 0) {
                //     buffer[len] = '\0';
                //     RCLCPP_INFO(this->get_logger(), "Received: '%s' from %s:%d len=%d",
                //                 buffer,
                //                 inet_ntoa(sender.sin_addr),
                //                 ntohs(sender.sin_port), len);
                // }

                mavlink_status_t status;
                mavlink_message_t msg;  // This is your received MAVLink message
                for (ssize_t i = 0; i < len; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                        // Successfully parsed a MAVLink message
                        //std::cout << "Received MAVLink message ID: " << msg.msgid << std::endl;

                        // Example: decode MANUAL_CONTROL message
                        if (msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) {
                            mavlink_manual_control_t ctrl;
                            mavlink_msg_manual_control_decode(&msg, &ctrl);
                            using namespace std::chrono_literals;

                            bool lowGear = buttonProcessor.isPressed(JoyHoldButtons::LOW_GEAR, ctrl.buttons);
                            (*oncePressButtons_)[JoyHoldButtons::LOW_GEAR]->SetValue(lowGear);
                            
                            bool reverseGear = buttonProcessor.isPressed(JoyHoldButtons::REVERSE_GEAR, ctrl.buttons);
                            (*oncePressButtons_)[JoyHoldButtons::REVERSE_GEAR]->SetValue(reverseGear); 

                            bool highGear = buttonProcessor.isPressed(JoyHoldButtons::HIGH_GEAR, ctrl.buttons);
                            (*oncePressButtons_)[JoyHoldButtons::HIGH_GEAR]->SetValue(highGear);                            

                            bool neutralGear = buttonProcessor.isPressed(JoyHoldButtons::NEUTRAL_GEAR, ctrl.buttons);
                            (*oncePressButtons_)[JoyHoldButtons::NEUTRAL_GEAR]->SetValue(neutralGear);

                            bool parkingGear = buttonProcessor.isPressed(JoyHoldButtons::PARKING_GEAR, ctrl.buttons);
                            (*oncePressButtons_)[JoyHoldButtons::PARKING_GEAR]->SetValue(parkingGear);

                            bool wdMode = buttonProcessor.isPressed(JoyHoldButtons::WD_MODE, ctrl.buttons);
                            (*oncePressButtons_)[JoyHoldButtons::WD_MODE]->SetValue(wdMode);
                            
                            bool enableJoy = buttonProcessor.isPressed(JoyHoldButtons::ENABLE, ctrl.buttons);
                            (*oncePressButtons_)[JoyHoldButtons::ENABLE]->SetValue(enableJoy);

                            if ((*oncePressButtons_)[JoyHoldButtons::LOW_GEAR]->IsOncePressed()) {
                                std::cout << "lowGear pressed!!" << std::endl;
                                //(*oncePressButtons_)[JoyHoldButtons::LOW_GEAR]->Reset();
                                //std::cout << "lowGear reset!!" << std::endl;
                            }
                            
                            if ((*oncePressButtons_)[JoyHoldButtons::HIGH_GEAR]->IsOncePressed()) {
                                std::cout << "highGear pressed!!" << std::endl;
                                //(*oncePressButtons_)[JoyHoldButtons::HIGH_GEAR]->Reset();
                                //std::cout << "highGear reset!!" << std::endl;
                            }

                            if ((*oncePressButtons_)[JoyHoldButtons::NEUTRAL_GEAR]->IsOncePressed()) {
                                std::cout << "neutralGear!!" << std::endl;
                                //(*oncePressButtons_)[JoyHoldButtons::NEUTRAL_GEAR]->Reset();
                                //std::cout << "neutralGear reset!!" << std::endl;
                            }

                            if ((*oncePressButtons_)[JoyHoldButtons::PARKING_GEAR]->IsOncePressed()) {
                                std::cout << "parkingGear!!" << std::endl;
                                //(*oncePressButtons_)[JoyHoldButtons::PARKING_GEAR]->Reset();
                                //std::cout << "parkingGear reset!!" << std::endl;
                            }

                            if ((*oncePressButtons_)[JoyHoldButtons::REVERSE_GEAR]->IsOncePressed()) {
                                std::cout << "reverseGear!!" << std::endl;
                                //(*oncePressButtons_)[JoyHoldButtons::REVERSE_GEAR]->Reset();
                                //std::cout << "reverseGear reset!!" << std::endl;
                            }

                            if ((*oncePressButtons_)[JoyHoldButtons::WD_MODE]->IsOncePressed()) {
                                std::cout << "wdMode!!" << std::endl;
                                //(*oncePressButtons_)[JoyHoldButtons::WD_MODE]->Reset();
                                //std::cout << "wdMode reset!!" << std::endl;
                            }
                            
                            // std::cout << "Mavlink struct:" << std::endl;
                            // std::cout << "axis Steering:" << ctrl.x << std::endl;
                            // std::cout << "axis Throttle:" << ctrl.y << std::endl;
                            // std::cout << "axis Break:" << ctrl.z << std::endl;
                            // std::cout << "axis Steering [ALT]:" << ctrl.r << std::endl;
                            // std::cout << "axis TURN LEFT | TURN RIGHT | AXIS_STEER_MULT_2:" << ctrl.aux1 << std::endl;
                            // std::cout << "axis AXIS_BRAKE_PRECHARGET | AXIS_STEER_MULT_1:" << ctrl.aux2 << std::endl;
                            // std::cout << "axis RESERVED a[1]:" << ctrl.aux3 << std::endl;
                            // std::cout << "axis RESERVED a[4]:" << ctrl.aux4 << std::endl;
                            // std::bitset<16> binary(ctrl.buttons);
                            // std::cout << "buttons: " << binary.to_string() << std::endl;


                            rclcpp::Time curr = now();
                            rclcpp::Duration time_diff = curr - prev_time_;
                            prev_time_ = curr;
                            int64_t millis = time_diff.nanoseconds() / 1000000;

                            ButtonMaskProcessor button_proc;
                            auto joy_msg = sensor_msgs::msg::Joy();
                            joy_msg.header.stamp = this->now();
                            joy_msg.header.frame_id = "joy_frame_polaris";
                            joy_msg.axes.resize(8);
                            joy_msg.axes[0] = ctrl.r / 1000.0; // Steering [ALT]
                            joy_msg.axes[1] = ctrl.aux3 / 1000.0; // RESERVED
                            joy_msg.axes[2] = ctrl.z / 1000.0; // Break
                            joy_msg.axes[3] = ctrl.x / 1000.0; // Steering
                            joy_msg.axes[4] = ctrl.aux4 / 1000.0; // Reserved
                            joy_msg.axes[5] = ctrl.y / 1000.0; // Throttle
                            joy_msg.axes[6] = ctrl.aux1 / 1000; // TURN LEFT | TURN RIGHT | AXIS_STEER_MULT_2
                            joy_msg.axes[7] = ctrl.aux2 / 1000; // AXIS_BRAKE_PRECHARGET | AXIS_STEER_MULT_1
                            joy_msg.buttons.resize(11);
                            if ((*oncePressButtons_)[JoyHoldButtons::LOW_GEAR]->IsOncePressed()) {
                                joy_msg.buttons[0] = 1;
                            }
                            if ((*oncePressButtons_)[JoyHoldButtons::REVERSE_GEAR]->IsOncePressed()) {
                                joy_msg.buttons[1] = 1;
                            }
                            if ((*oncePressButtons_)[JoyHoldButtons::PARKING_GEAR]->IsOncePressed()) {
                                joy_msg.buttons[2] = 1;
                            }
                            if ((*oncePressButtons_)[JoyHoldButtons::HIGH_GEAR]->IsOncePressed()) {
                                joy_msg.buttons[3] = 1;
                            }
                            if (button_proc.isPressed(JoyHoldButtons::LED, ctrl.buttons)) {
                                joy_msg.buttons[4] = 1;
                            }
                            if ((*oncePressButtons_)[JoyHoldButtons::NEUTRAL_GEAR]->IsOncePressed()) {
                                joy_msg.buttons[5] = 1;
                            }

                            if (button_proc.isPressed(JoyHoldButtons::WIPER, ctrl.buttons)) {
                                joy_msg.buttons[6] = 1;
                            }
                            if ((*oncePressButtons_)[JoyHoldButtons::ENABLE]->IsOncePressed()) {
                                joy_msg.buttons[7] = 1;
                            }

                            if ((*oncePressButtons_)[JoyHoldButtons::WD_MODE]->IsOncePressed()) {
                                joy_msg.buttons[8] = 1;
                            }


                            joy_publisher_->publish(joy_msg);

                            //std::cout << "Seq ID: " << ctrl.aux5 << ", Source time diff (millis)=" << ctrl.aux6 << ", Mavlink time diff=" << millis << ", latTime=" << abs(ctrl.aux6 - millis) << std::endl;
                        }
                    }
                }
                
            }

        }
    

    int sock_;
    rclcpp::Time prev_time_;
    struct sockaddr_in server_addr_{};
    std::thread recv_thread_;
    std::shared_ptr<std::map<JoyHoldButtons, std::shared_ptr<ButtonValue>>> oncePressButtons_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;

    //   rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    //   rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;

};
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<std::map<JoyHoldButtons, std::shared_ptr<ButtonValue>>> oncePress_button_map = std::make_shared<std::map<JoyHoldButtons, std::shared_ptr<ButtonValue>>>();
  auto lowGear = std::make_shared<ButtonValue>();
  auto highGear = std::make_shared<ButtonValue>();
  auto neutralGear = std::make_shared<ButtonValue>();
  auto parkingGear = std::make_shared<ButtonValue>();
  auto reverseGear = std::make_shared<ButtonValue>();
  auto wdMode = std::make_shared<ButtonValue>(); // 2WD/4WD
  auto enable_joy = std::make_shared<ButtonValue>();

  (*oncePress_button_map)[JoyHoldButtons::LOW_GEAR] = lowGear;
  (*oncePress_button_map)[JoyHoldButtons::HIGH_GEAR] = highGear;
  (*oncePress_button_map)[JoyHoldButtons::NEUTRAL_GEAR] = neutralGear;
  (*oncePress_button_map)[JoyHoldButtons::PARKING_GEAR] = parkingGear;
  (*oncePress_button_map)[JoyHoldButtons::REVERSE_GEAR] = reverseGear;
  (*oncePress_button_map)[JoyHoldButtons::WD_MODE] = wdMode;
  (*oncePress_button_map)[JoyHoldButtons::ENABLE] = enable_joy;

  auto lowGearWatch_node = std::make_shared<ButtonPressed>("LowGearWatch", lowGear);
  auto highGearWatch_node = std::make_shared<ButtonPressed>("HighGearWatch", highGear);
  auto neutralGearWatch_node = std::make_shared<ButtonPressed>("NeutralGearWatch", neutralGear);
  auto parkingGearWatch_node = std::make_shared<ButtonPressed>("ParkingGearWatch", parkingGear);
  auto reverseGearWatch_node = std::make_shared<ButtonPressed>("ReverseGearWatch", reverseGear);
  auto wdModeWatch_node = std::make_shared<ButtonPressed>("WDModeWatch", wdMode);
  auto enableJoy_node = std::make_shared<ButtonPressed>("EnableJoyWatch", enable_joy);
  auto takTelemetry_node = std::make_shared<TAK_telemetry>();
  auto joyReader_node = std::make_shared<JoyReader>(oncePress_button_map);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(lowGearWatch_node);
  executor.add_node(highGearWatch_node);
  executor.add_node(neutralGearWatch_node);
  executor.add_node(parkingGearWatch_node);
  executor.add_node(reverseGearWatch_node);
  executor.add_node(wdModeWatch_node);
  executor.add_node(enableJoy_node);
  executor.add_node(takTelemetry_node);

  executor.add_node(joyReader_node);

  executor.spin();
  //rclcpp::spin(std::make_shared<JoyReader>());
  rclcpp::shutdown();
  return 0;
}