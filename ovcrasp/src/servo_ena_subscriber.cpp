#include <memory>   

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "std_msgs/msg/float32_multi_array.hpp"
#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"

using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::StopBits;

class ServoSubscriber : public rclcpp::Node
{
public:
  ServoSubscriber() : Node("servo_subscriber"), io_context_(1)
  {
    SerialPortConfig config(115200, FlowControl::NONE, Parity::NONE, StopBits::ONE);
    driver_ = std::make_unique<SerialDriver>(io_context_);
    driver_->init_port("/dev/ttyUSB2", config);
    driver_->port()->open();
    
    auto topic_callback = 
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        if (msg->data != last_msg_){
          last_msg_ = msg->data;
          std::vector<uint8_t> buffer(msg->data.begin(), msg->data.end());
          driver_->port()->send(buffer);
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        }
      };
      subscription_ = 
        this->create_subscription<std_msgs::msg::String>("servo_ena_topic", 10, topic_callback);
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  IoContext io_context_;
  std::unique_ptr<SerialDriver> driver_;
  std::string last_msg_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoSubscriber>());
    rclcpp::shutdown();
    return 0;
}