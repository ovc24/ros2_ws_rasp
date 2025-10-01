#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"

using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::StopBits;

using namespace std::chrono_literals;

class EncodersPublisher : public rclcpp::Node
{
public:
  EncodersPublisher() : Node("encoders_publisher"), io_context_(1)
  {
    SerialPortConfig config(115200, FlowControl::NONE, Parity::NONE, StopBits::ONE);
    driver_ = std::make_unique<SerialDriver>(io_context_);
    driver_->init_port("/dev/ttyUSB0",config);
    driver_->port()->open();
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("encoders_msgs",10);
    //publisher_ = this->create_publisher<std_msgs::msg::String>("encoders_msgs",10);
    timer_ = this->create_wall_timer(100ms, std::bind(&EncodersPublisher::read_serial, this));
  }
private:
  void read_serial()
  {
    std::vector<uint8_t> buffer(256);
    size_t n = driver_->port()->receive(buffer);

    if(n>0){
      std::string raw(buffer.begin(), buffer.begin()+n);
      leftover_ += raw;
      size_t pos = 0;

      while((pos = leftover_.find('\n')) != std::string::npos){
        std::string line = leftover_.substr(0, pos);
        leftover_.erase(0, pos + 1);
        
        line.erase(0, line.find_first_not_of(" {"));
        line.erase(line.find_last_not_of(" }\r") + 1);

        std::vector<float> values;
        std::stringstream ss(line);
        std::string item;
        while (std::getline(ss, item, ',')){
          try {
            values.push_back(std::stof(item));
          } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Valor invalido en linea: %s", item.c_str());
          }
        }
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = values;
        // std_msgs::msg::String msg;
        // msg.data = line;
        publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Recibido: %s", line.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "No data received");
    }
  }
  IoContext io_context_;
  //std::unique_ptr<SerialPortConfig> config_;
  std::unique_ptr<SerialDriver> driver_;  
  rclcpp::TimerBase::SharedPtr timer_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  std::string leftover_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncodersPublisher>());
    rclcpp::shutdown();
    return 0;
}