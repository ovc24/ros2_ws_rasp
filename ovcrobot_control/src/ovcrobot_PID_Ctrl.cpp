#include <memory>   
#include <vector>
#include <chrono>
#include <algorithm>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ovcrobot_interfaces/action/set_joint_target.hpp"

using namespace std::chrono_literals;

class ControlPID : public rclcpp::Node
{
public:
  using SetJointTarget = ovcrobot_interfaces::action::SetJointTarget;
  using GoalHandleSetJointTarget = rclcpp_action::ServerGoalHandle<SetJointTarget>;

  ControlPID() : Node("ovcrobot_PID")
  {
    kp_ = this->declare_parameter<std::vector<double>>("kp", std::vector<double>(6,0.0));
    ki_ = this->declare_parameter<std::vector<double>>("ki", std::vector<double>(6,0.0));
    kd_ = this->declare_parameter<std::vector<double>>("kd", std::vector<double>(6,0.0));
    dt_ = this->declare_parameter<double>("dt", 0.1);
    max_step_ = this->declare_parameter<double>("max_step", 3.0);

    for (size_t i =0; i<6; ++i){
      RCLCPP_INFO(this->get_logger(), "Motor %zu -> Kp: %f, Ki: %f, Kd: %f",i+1, 
                  this->get_parameter("kp").as_double_array()[i],
                  this->get_parameter("ki").as_double_array()[i],
                  this->get_parameter("kd").as_double_array()[i]);
    }

    integral_.resize(6,0.0);
    prev_error_.resize(6,0.0);
    encoders_measure_.resize(6,0.0);
    target_.resize(6,0.0);
    target_active_.resize(6,0.0);

    target_received_ = false;
    angMin_ = {-50,-60,-60,-90,-25, -180};
    angMax_ = { 50, 60, 80, 90,120, 180};

    //max_step = 30.0;

    sub_encoders_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("encoders_msgs", 10, std::bind(&ControlPID::encoders_callback, this,std::placeholders::_1));
    //sub_target_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("target_sp", 10, std::bind(&ControlPID::target_sp_callback, this,std::placeholders::_1));
    pub_motors_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motores_topic",10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(dt_ * 1000.0)),
        std::bind(&ControlPID::control_pid_callback, this));

    action_server_ = rclcpp_action::create_server<SetJointTarget>(this, "set_joint_target", std::bind(&ControlPID::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                                                                            std::bind(&ControlPID::handle_cancel, this, std::placeholders::_1),
                                                                                            std::bind(&ControlPID::handle_accepted, this, std::placeholders::_1));
  }

private:
  
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const SetJointTarget::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Nueva meta recibida");
    if (goal->target.size() != 6) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSetJointTarget>)
  {
    RCLCPP_INFO(this->get_logger(), "Cancelando meta");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSetJointTarget> goal_handle)
  {
    target_received_ = true;
    std::thread([this, goal_handle](){
      auto goal = goal_handle->get_goal();

      for (size_t i=0; i<6; i++){
        target_[i] = std::clamp(goal->target[i], angMin_[i], angMax_[i]);
        target_active_[i] = encoders_measure_[i];
      }
      
      auto feedback = std::make_shared<SetJointTarget::Feedback>();
      auto result = std::make_shared<SetJointTarget::Result>();

      rclcpp::Rate rate(10);
      while (rclcpp::ok()) {
        if (goal_handle->is_canceling()){
          result->success = false;
          goal_handle->canceled(result);
          return;
        }
        for (int i=0; i<6;i++){
          feedback->current[i] = encoders_measure_[i];
        }
        
        goal_handle->publish_feedback(feedback);

        double total_err = 0.0;
        for (size_t i=0; i<6; ++i)
          total_err += std::fabs(target_[i] - encoders_measure_[i]);
        if(total_err<4.0){
          result->success = true;
          goal_handle->succeed(result);
          return;
        }

        // for (size_t i =0; i<6;i++){
        //   double diff = target_[i] - target_active_[i];
        //   if(std::fabs(diff)>max_step_){
        //     target_active_[i] += max_step_ * (diff > 0 ? 1 : -1);
        //     //RCLCPP_INFO(this->get_logger(), "Motor %zu: diff = %f", i, target_active_[i]);
        //   }else{
        //     target_active_[i] = target_[i];
        //   }
        // }
     
        rate.sleep();
      }
    }).detach();
  }

  void encoders_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
      for(size_t i=0;i<6;++i){
        encoders_measure_[i] = msg->data[i];
      }
    }
  // void target_sp_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  // {
  //   if(msg->data.size()<6) return;
  //   if(msg->data.size()>6) return;
  //   for(size_t i=0;i<6;++i){
  //     target_[i] = std::clamp(msg->data[i], angMin_[i], angMax_[i]);
  //     target_active_[i] = encoders_measure_[i];
  //   }
  //   target_received_ = true;
  // }
  void control_pid_callback()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(6);
    for (size_t i = 0; i<6; ++i)
    {
      double diff = target_[i] - target_active_[i];

      if(std::fabs(diff)>max_step_){
        target_active_[i] += max_step_ * (diff > 0 ? 1 : -1);
        //RCLCPP_INFO(this->get_logger(), "Motor %zu: diff = %f", i, target_active_[i]);
      }else{
        target_active_[i] = target_[i];
      }
      double error = target_active_[i] - encoders_measure_[i];
      //double error = target_[i] - encoders_measure_[i];
      integral_[i] += error * dt_;
      double derivative = (error - prev_error_[i]) / dt_;
      prev_error_[i] = error;

      msg.data[i] = kp_[i] * error + ki_[i] * integral_[i] + kd_[i] * derivative;
    }
    //RCLCPP_INFO(this->get_logger(), "%f, %f, %f, %f, %f, %f", msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
    
    if(target_received_) pub_motors_->publish(msg);
  }
  double dt_;
  float max_step_;
  bool target_received_ = false;
  std::vector<double> kp_, ki_, kd_;
  std::vector<double> integral_, prev_error_;
  std::vector<float> encoders_measure_, target_, target_active_;
  std::vector<float> angMin_, angMax_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_encoders_;
  //rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_target_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motors_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<SetJointTarget>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlPID>());
    rclcpp::shutdown();
    return 0;
}