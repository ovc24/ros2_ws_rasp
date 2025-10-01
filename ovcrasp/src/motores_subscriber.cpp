#include <memory>   

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"

using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::StopBits;

class MotoresSubscriber : public rclcpp::Node
{
public:
  MotoresSubscriber() : Node("motores_subscriber"), io_context_(1)
  {
    SerialPortConfig config(115200, FlowControl::NONE, Parity::NONE, StopBits::ONE);
    driver_ = std::make_unique<SerialDriver>(io_context_);
    driver_->init_port("/dev/ttyUSB1", config);
    driver_->port()->open();
    
    auto topic_callback = 
      [this](std_msgs::msg::Float32MultiArray::UniquePtr msg) -> void {
        std::ostringstream oss;
        for (size_t i=0; i<msg->data.size();i++){
          oss << msg->data[i];
          if(i < msg->data.size() - 1){
            oss << ",";
          }
        }
        oss << "\n";
        std::string out = oss.str();
        std::vector<uint8_t> buffer(out.begin(), out.end());
        driver_->port()->send(buffer);
        // RCLCPP_INFO(this->get_logger(), "I heard: %s", out.c_str());
      };
      subscription_ = 
        this->create_subscription<std_msgs::msg::Float32MultiArray>("motores_topic", 10, topic_callback);
  }
private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  IoContext io_context_;
  std::unique_ptr<SerialDriver> driver_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoresSubscriber>());
    rclcpp::shutdown();
    return 0;
}