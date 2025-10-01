#include <memory>
#include <vector>
#include <chrono>
#include <sstream>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ovcrobot_interfaces/action/set_joint_target.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"


using namespace std::chrono_literals;

class ActionClientBridge : public rclcpp::Node
{
 public:
   using SetJointTarget = ovcrobot_interfaces::action::SetJointTarget;
   using GoalHandleSetJointTarget = rclcpp_action::ClientGoalHandle<SetJointTarget>;
   
   ActionClientBridge() : Node("action_client_bridge")
   {
    
    // double pi = M_PI;
    action_client_ = rclcpp_action::create_client<SetJointTarget>(this,"set_joint_target");

    //pub_feedback_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("targetSP_feedback", 10);
    pub_result_ = this->create_publisher<std_msgs::msg::Bool>("targetSP_result",10);

    sub_dk_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("targetDK", 10, std::bind(&ActionClientBridge::dk_callback, this, std::placeholders::_1));
    sub_ik_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("targetIK", 10 , std::bind(&ActionClientBridge::ik_callback, this, std::placeholders::_1));
    ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    // while(!ik_client_->wait_for_service(1s)){
    //     RCLCPP_INFO(this->get_logger(), "Esperando /compute_ik");
    // }
    last_target_.resize(6,std::numeric_limits<float>::quiet_NaN());
   }
//    void init_move_group()
//    {
//     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),"ovcrobot");
//    }
 private:
   void dk_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
   {
    if (msg->data.size() != 6) {
        // RCLCPP_WARN(this->get_logger(), "DK message con tamaño incorrecto (%zu)", msg->data.size());
        return;
    }
    //send_goal(msg->data);
    
    if (msg->data != last_target_)
     {
        send_goal(msg->data);
    }
   }

   void ik_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
   {
    // RCLCPP_INFO(this->get_logger(), "Recibida pose IK, intentando resolver...");
    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = "ovcrobot";
    request->ik_request.pose_stamped = *msg;

    ik_client_->async_send_request(request,
        [this](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future){
            auto response = future.get();
            if(response->error_code.val == 1){
                // RCLCPP_INFO(this->get_logger(), "IK resuelto correctamente!");
                std::vector<float> joint_positions_float;
                for (auto & val : response->solution.joint_state.position) {
                    joint_positions_float.push_back(static_cast<float>(val*180/M_PI));
                    // RCLCPP_INFO(this->get_logger(), "  %.4f", val*180/M_PI);
                }
                send_goal(joint_positions_float);
            } else {
                // RCLCPP_ERROR(this->get_logger(), "Fallo en la IK");
            }
        });
 
   }

   void send_goal(const std::vector<float> &joint_values)
   {
    if (active_goal_) {
        // RCLCPP_WARN(this->get_logger(), "Goal en curso, ignorando nuevo target");
        return; // ignoramos mientras haya un goal activo
    }
    if(!action_client_->wait_for_action_server(1s)){
        RCLCPP_ERROR(this->get_logger(), "Action server no disponible");
        return;
    }
    if (joint_values.size() != 6){
        RCLCPP_WARN(this->get_logger(), "Joint values tamano incorrecto (%zu)", joint_values.size());
           return; 
    }
    auto goal_msg = SetJointTarget::Goal();
    
    for (int i=0; i<6; i++) goal_msg.target[i] = joint_values[i];

    // std_msgs::msg::Bool moving_msg;
    // moving_msg.data = false;
    // pub_result_->publish(moving_msg);

    auto send_goal_options = rclcpp_action::Client<SetJointTarget>::SendGoalOptions();
    
    //send_goal_options.result_callback = std::bind(&ActionClientBridge::result_callback, this, std::placeholders::_1);

    send_goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleSetJointTarget> goal_handle){
        if (goal_handle){
            active_goal_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal aceptado por el servidor");
            std_msgs::msg::Bool msg;
            msg.data = false;
            pub_result_->publish(msg);
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal rechazado por el servidor");
        }
    };

    send_goal_options.result_callback = [this, joint_values](const GoalHandleSetJointTarget::WrappedResult &result){
        active_goal_ = nullptr;
        std_msgs::msg::Bool msg;
        msg.data = result.result->success;
        pub_result_->publish(msg);

        if(result.result->success)
        {
          last_target_ = joint_values;
        //   std_msgs::msg::Bool msg;
        //   msg.data = result.result->success;
        //   pub_result_->publish(msg);
            // if(msg.data){
          RCLCPP_INFO(this->get_logger(), "Action completada con exito");
        }else
        {
          RCLCPP_INFO(this->get_logger(), "Action fallida");
        }
        };
        //std_msgs::msg::Bool msg;
        // msg.data = result.result->success;
        // pub_result_->publish(msg);
        // if(msg.data){
        //     RCLCPP_INFO(this->get_logger(), "Action completada con exito");
        // }else{
        //     RCLCPP_INFO(this->get_logger(), "Action fallida");
        // }
   
    RCLCPP_INFO(this->get_logger(), "Enviando goal a targetSP ...");
    action_client_->async_send_goal(goal_msg, send_goal_options);
   }

//    void result_callback(const GoalHandleSetJointTarget::WrappedResult &result)
//    {
//     active_goal_ = nullptr;
//     std_msgs::msg::Bool msg;
//     msg.data = result.result->success;

//     if(msg.data){
//         RCLCPP_INFO(this->get_logger(), "Action completada con exito");
//     }else{
//         RCLCPP_INFO(this->get_logger(), "Action fallida");
//     }

//     pub_result_->publish(msg);
//    }
   rclcpp_action::Client<SetJointTarget>::SharedPtr action_client_;
   rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_dk_;
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ik_;
   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_result_;
   std::shared_ptr<GoalHandleSetJointTarget> active_goal_ = nullptr;
   std::vector<float> last_target_;
   rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
//    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClientBridge>();
    // node->init_move_group();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}