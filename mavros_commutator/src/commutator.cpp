#include <mavlink/v2.0/common/mavlink.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
// #include "mavros_msgs/msg/manual_control.hpp"
// #include "mavros_msgs/msg/mavlink.hpp"

#include "ds_dbw_msgs/msg/gear_report.hpp"
#include "ds_dbw_msgs/msg/system_report.hpp"


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <bitset>
#include <map>
#include <fcntl.h>
#include <libgpsmm.h>
#include <iostream>

struct GearState {
    uint8_t gear_current;
    uint8_t gear_pending;
    uint8_t gear_reject;
    uint8_t gear_cmd;
    bool gear_ready;
    bool gear_fault;
};

enum class JoyHoldButtons { // MAPPING struct
    LOW_GEAR = 1 << 0, // A button
    REVERSE_GEAR = 1 << 1, // B button
    PARKING_GEAR = 1 << 2, // X button
    HIGH_GEAR = 1 << 3, // Y button
    NEUTRAL_GEAR = 1 << 5, // Top-right front button
    WD_MODE = 1 << 8, // 'Logitech' central button
    ENABLE = 1 << 7, // Start
    LED = 1 << 4, // LEDs not holded
    WIPER = 1 << 6, // WIPER not holded
    NONE = 1 << 16 // NONE - undetermined value
};

std::string to_string(JoyHoldButtons button) {
    switch (button) {
        case JoyHoldButtons::LOW_GEAR: return "LOW_GEAR";
        case JoyHoldButtons::REVERSE_GEAR: return "REVERSE_GEAR";
        case JoyHoldButtons::PARKING_GEAR: return "PARKING_GEAR";
        case JoyHoldButtons::HIGH_GEAR: return "HIGH_GEAR";
        case JoyHoldButtons::LED: return "LED";
        case JoyHoldButtons::NEUTRAL_GEAR: return "NEUTRAL_GEAR";
        case JoyHoldButtons::WIPER: return "WIPER";
        case JoyHoldButtons::ENABLE: return "ENABLE";
        case JoyHoldButtons::WD_MODE: return "WD_MODE";
        case JoyHoldButtons::NONE: return "NONE";
        default: return "UNKNOWN_BUTTON";
    }
}


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

        bool operator<(const ButtonValue& other) const {
            return last_update_time_ < other.last_update_time_;
        }

        bool operator<=(const ButtonValue& other) const {
            return last_update_time_ <= other.last_update_time_;
        }

        bool operator>(const ButtonValue& other) const {
            return last_update_time_ > other.last_update_time_;
        }

        bool operator>=(const ButtonValue& other) const {
            return last_update_time_ >= other.last_update_time_;
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
        }

        bool IsOncePressed() {
        // Returns the current IsOncePressed status
            return pressed_;
        }

        void SetPressed(const rclcpp::Time& curr_time) {
        // Set IsOncePressed = true if a button holding time was reached
            pressed_ = true;
            last_update_time_ = curr_time;
        }



        //rclcpp::Time update_time;
        rclcpp::Time last_update_time_;
    private:
        bool value_;
        bool pressed_;
        //rclcpp::Time last_update_time_;
        

};

class UvgGearMonitor: public rclcpp::Node {
    public:
        explicit UvgGearMonitor(std::shared_ptr<std::map<JoyHoldButtons, std::shared_ptr<ButtonValue>>> oncePressButtons, std::shared_ptr<GearState> gear_state) : Node("UvgMonitor") {
            
            RCLCPP_INFO(this->get_logger(), "UvgGearMonitor '%s' node started.", this->get_name());

            gear_report_sub_ = this->create_subscription<ds_dbw_msgs::msg::GearReport>("/vehicle/gear/report", 1,
                                std::bind(&UvgGearMonitor::gear_report_callback, this, std::placeholders::_1));

            oncePressButtons_ = oncePressButtons;

            gear_state_ = gear_state; // std::make_shared<GearState>();
            gear_state_->gear_pending = ds_dbw_msgs::msg::Gear::PARK;

            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UvgGearMonitor::check_state_, this));

        }


    private:
        void check_state_(){ // check state accordingly to the pending gear, and reset the button
            // check last time actual data arrived from UGV
            if (gear_report_sub_->get_publisher_count() == 0) {
            //if ((this->now().seconds() - gear_state_->last_update_time_sec) >= 0.05) {
                 RCLCPP_INFO(this->get_logger(), "Are '/vehicle/gear/report' msgs published ('%s')?", this->get_name());

                 // reset pending gear
                 gear_state_->gear_pending = ds_dbw_msgs::msg::Gear::NONE;
            }

            if (gear_state_->gear_current == gear_state_->gear_pending) { // UVG is trying to switch the gear
                if (gear_state_->gear_reject == ds_dbw_msgs::msg::GearReject::NONE) { // no problems with gear switching
                    if (!gear_state_->gear_fault) { // no problems with gear switching
                    //if (gear_state_->gear_current == gear_state_->gear_pending) { // if the gear is already active
                        
                        // determine the button 

                        JoyHoldButtons key;
                        switch (gear_state_->gear_current) {
                            case ds_dbw_msgs::msg::Gear::PARK:
                                key = JoyHoldButtons::PARKING_GEAR;
                                break;

                            case ds_dbw_msgs::msg::Gear::REVERSE:
                                key = JoyHoldButtons::REVERSE_GEAR;
                                break;

                            case ds_dbw_msgs::msg::Gear::NEUTRAL:
                                key = JoyHoldButtons::NEUTRAL_GEAR;
                                break;
                            
                            case ds_dbw_msgs::msg::Gear::DRIVE:
                                key = JoyHoldButtons::HIGH_GEAR;
                                break;

                            case ds_dbw_msgs::msg::Gear::LOW:
                                key = JoyHoldButtons::LOW_GEAR;
                                break;  
                            
                            default:
                                key = JoyHoldButtons::NONE;
                        }
                        // reset the button!
                        if (key != JoyHoldButtons::NONE) {
                            if ((*oncePressButtons_)[key]->IsOncePressed()) {
                                (*oncePressButtons_)[key]->Reset();
                            }
                        }
                    }
                }
            }

        }


        void gear_report_callback(const std::shared_ptr<const ds_dbw_msgs::msg::GearReport>& msg) {
            //RCLCPP_INFO(this->get_logger(), "gear_report_callback '%s'", this->get_name());
            gear_state_->gear_current = msg->gear.value;
            gear_state_->gear_reject = msg->reject.value;
            gear_state_->gear_cmd = msg->cmd.value;
            gear_state_->gear_ready = msg->ready;
            gear_state_->gear_fault = msg->fault;
        }
        
        rclcpp::Subscription<ds_dbw_msgs::msg::GearReport>::SharedPtr gear_report_sub_;
        std::shared_ptr<GearState> gear_state_;
        std::shared_ptr<std::map<JoyHoldButtons, std::shared_ptr<ButtonValue>>> oncePressButtons_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time last_msg_arrived_;
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
        uint16_t prev_value_ = 0;
        rclcpp::Time onButton_time_ = this->now();
        rclcpp::Time change_time_ = this->now();
        double hold_time_ = 2.0; // secs
        bool enabled_ = false;
         

        void process_value() {
            uint16_t curr_value = button_value_->GetValue();
   
            if (curr_value != prev_value_) {
                change_time_ =  this->now();
                if (curr_value == 1) {
                    enabled_ = true;
                } else {
                    enabled_ = false;
                }
                prev_value_ = curr_value;
                
            }
            

            rclcpp::Duration time_diff = this->now() - change_time_;
            double seconds = time_diff.seconds();
            if (seconds > 30.0) change_time_ =  this->now(); // reset clock

            if (seconds >= hold_time_) {
                
                if (curr_value == 1 && enabled_) {
                    if (!button_value_->IsOncePressed()) {
                        button_value_->SetPressed(this->now());
                        
                    } else {
                        button_value_->Reset();
                    }
                    enabled_ = false;
                }
            }

             

        }
};



class JoyReader : public rclcpp::Node {
    public:
    JoyReader(std::shared_ptr<std::map<JoyHoldButtons, std::shared_ptr<ButtonValue>>> oncePressButtons, std::shared_ptr<GearState> gear_state) : Node("joy_reader") {
        // The Node listening maviklink MANUAL_CONTROL messages, then publishes appropriate sensor_msgs::msg::Joy messages under /joy_polaris ROS topic.

        RCLCPP_INFO(this->get_logger(), "Joystick commands reader node started.");

        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_INFO(this->get_logger(), "Socket creation failed.");
        }

        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(14550);  // Target port (e.g. QGroundControl or autopilot)
        //inet_pton(AF_INET, "172.25.64.194", &server_addr_.sin_addr);

        server_addr_.sin_addr.s_addr = INADDR_ANY;  // Target IP

        if (bind(sock_, (const struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
            RCLCPP_INFO(this->get_logger(), "Socket bind failed.");
            close(sock_);
        }

        // unblock sock
        int flags = fcntl(sock_, F_GETFL, 0);
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

        oncePressButtons_ = oncePressButtons;
        gear_state_ = gear_state;

        // checking override command from the UGV
        system_report_sub_ = this->create_subscription<ds_dbw_msgs::msg::SystemReport>("/vehicle/system/report", 10,
                std::bind(&JoyReader::system_report_callback, this, std::placeholders::_1));

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

        void system_report_callback(const ds_dbw_msgs::msg::SystemReport::SharedPtr msg) {
            // checks the override state
            if (msg->override) {
                for (const auto& [key, valuePtr] : *oncePressButtons_) {
                    if (key == JoyHoldButtons::ENABLE) {
                        (*oncePressButtons_)[key]->Reset(); // reset enable buttnon
                    }
                }
            }
        }

        void filter_pending_gear () {
            // filters only a single pending gear if multiple buttons are active or the gear is in place

            /* find the latest (by time) holding button */
            auto latest = (*oncePressButtons_).begin()->first;
            bool present = false;
            for (const auto& [key, valuePtr] : *oncePressButtons_) {
                if (key == JoyHoldButtons::WD_MODE || key == JoyHoldButtons::ENABLE) continue; // ignore not GEAR buttons
                if (!(*oncePressButtons_)[key]->IsOncePressed()) continue; // ignore not active GEAR buttons
                latest = key;
                present = true;
                break;
            }
            if (!present) return; // there are not any GEAR button active, return


            for (const auto& [key, valuePtr] : *oncePressButtons_) {
                if (key == JoyHoldButtons::WD_MODE || key == JoyHoldButtons::ENABLE) continue; // ignore not GEAR buttons
                if (!(*oncePressButtons_)[key]->IsOncePressed()) continue; // ignore not active GEAR buttons

                if (*(*oncePressButtons_)[key] > *(*oncePressButtons_)[latest]) {
                    latest = key;
                }
            }

            for (const auto& [key, valuePtr] : *oncePressButtons_) {
                if (key == JoyHoldButtons::WD_MODE || key == JoyHoldButtons::ENABLE) continue; // ignore not GEAR buttons
                if (!(*oncePressButtons_)[key]->IsOncePressed()) continue; // ignore not active GEAR buttons
                
                if (key != latest) {
                    (*oncePressButtons_)[key]->Reset();
                    
                }

            }

            // Ser Pending GEAR
            switch (latest) {
                case JoyHoldButtons::PARKING_GEAR:
                    gear_state_->gear_pending = ds_dbw_msgs::msg::Gear::PARK;
                    break;

                case JoyHoldButtons::REVERSE_GEAR:
                    gear_state_->gear_pending = ds_dbw_msgs::msg::Gear::REVERSE;
                    break;

                case JoyHoldButtons::NEUTRAL_GEAR:
                    gear_state_->gear_pending = ds_dbw_msgs::msg::Gear::NEUTRAL;
                    break;
                
                case JoyHoldButtons::HIGH_GEAR:
                    gear_state_->gear_pending = ds_dbw_msgs::msg::Gear::DRIVE;
                    break;

                case JoyHoldButtons::LOW_GEAR:
                    gear_state_->gear_pending = ds_dbw_msgs::msg::Gear::LOW;
                    break;  
                
                default:
                    gear_state_->gear_pending = ds_dbw_msgs::msg::Gear::NONE;
            }

        }

 
        void receive_loop() {
            // RCLCPP_INFO(this->get_logger(), "receive_loop");
            uint8_t buffer[2048];
            sockaddr_in sender{};
            socklen_t sender_len = sizeof(sender);

            ButtonMaskProcessor buttonProcessor;

            while (rclcpp::ok()) {
                ssize_t len = recvfrom(sock_, buffer, sizeof(buffer) - 1, 0,
                                    (struct sockaddr*)&sender, &sender_len);
                if (len <= 0) {
                    //RCLCPP_INFO(this->get_logger(), "break");
                    break; //!!!!
                } else {
                    //RCLCPP_INFO(this->get_logger(), "Recieved: %d", len);
                }
 
                mavlink_status_t status{};
                mavlink_message_t msg;  // This is your received MAVLink message


                for (ssize_t i = 0; i < len; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {

                        if (msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) {
                            mavlink_manual_control_t ctrl;
                            mavlink_msg_manual_control_decode(&msg, &ctrl);
                            if (ctrl.target != 8) {
                                //RCLCPP_INFO(this->get_logger(), "Ignoring %d", ctrl.target);
                                continue;
                            }
                            //RCLCPP_INFO(this->get_logger(), "After continue");
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
                            

                            filter_pending_gear(); // choose only single GEAR to activate


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
    std::shared_ptr<GearState> gear_state_; // shared gear's state struct
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    rclcpp::Subscription<ds_dbw_msgs::msg::SystemReport>::SharedPtr system_report_sub_; 

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

  

  std::shared_ptr<GearState> gear_state = std::make_shared<GearState>();
  auto gearMonitor_node = std::make_shared<UvgGearMonitor>(oncePress_button_map, gear_state);
  auto joyReader_node = std::make_shared<JoyReader>(oncePress_button_map, gear_state);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(lowGearWatch_node);
  executor.add_node(highGearWatch_node);
  executor.add_node(neutralGearWatch_node);
  executor.add_node(parkingGearWatch_node);
  executor.add_node(reverseGearWatch_node);
  executor.add_node(wdModeWatch_node);
  executor.add_node(enableJoy_node);
  executor.add_node(gearMonitor_node);

  executor.add_node(joyReader_node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}