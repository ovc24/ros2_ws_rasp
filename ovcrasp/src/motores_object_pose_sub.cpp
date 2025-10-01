#include <memory>   

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MotoresObjectPoseSubscriber : public rclcpp::Node
{
public:
  MotoresObjectPoseSubscriber() : Node("motores_object_pose_sub")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("object_pose", 10, std::bind(&MotoresObjectPoseSubscriber::ObjectPose_callback, this,std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motores_topic",10);
  }
private:
  void ObjectPose_callback()
  {
    
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoresObjectPoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}