#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class EncodersJointPublisher : public rclcpp::Node
{
public:
  EncodersJointPublisher() : Node("encoders_joint_state_pub")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states_encoders",10);
 
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("encoders_msgs",10, std::bind(&EncodersJointPublisher::encoder_callback, this,std::placeholders::_1));

    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    RCLCPP_INFO(this->get_logger(), "Nodo encoders_joint_state_pub iniciado");
  }
private:
   void encoder_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
   {
    if(msg->data.size() != 6)
    {
        // RCLCPP_WARN(this->get_logger(), "Se recibio: %zu", msg->data.size());
        return;
    }
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = this-> now();
    joint_msg.name = joint_names_;
    joint_msg.position.clear();
    for (auto f : msg->data){joint_msg.position.push_back(static_cast<double>(f));}
    joint_msg.velocity = std::vector<double>(6,0.0);
    joint_msg.effort = std::vector<double>(6,0.0);

    publisher_->publish(joint_msg);
    // RCLCPP_INFO(this->get_logger(), "Publicando JointState: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", joint_msg.position[0], joint_msg.position[1],joint_msg.position[2],joint_msg.position[3],joint_msg.position[4],joint_msg.position[5]);
   }
   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
   rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
   std::vector<std::string> joint_names_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncodersJointPublisher>());
    rclcpp::shutdown();
    return 0;
}