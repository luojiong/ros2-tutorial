#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_demo/action/custom_action.hpp"

using namespace std::chrono_literals;

class ProgressActionClient : public rclcpp::Node
{
public:
  using CustomAction = ros2_demo::action::CustomAction;
  using GoalHandleCustomAction = rclcpp_action::ClientGoalHandle<CustomAction>;
  
  explicit ProgressActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("progress_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<CustomAction>(
      this, "progress_tracker");
      
    // 声明参数
    this->declare_parameter("order", 2);
    this->declare_parameter("target_value", 10.0);
    this->declare_parameter("auto_send", true);
    
    // 如果自动发送设置为true，则在启动时发送目标
    if (this->get_parameter("auto_send").as_bool()) {
      this->timer_ = this->create_wall_timer(
        500ms, std::bind(&ProgressActionClient::send_goal, this));
    }
  }
  
  void send_goal()
  {
    // 只发送一次，取消定时器
    this->timer_.reset();
    
    // 等待Action服务可用
    if (!this->client_ptr_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action服务器不可用");
      rclcpp::shutdown();
      return;
    }
    
    // 创建目标
    auto goal_msg = CustomAction::Goal();
    goal_msg.order = this->get_parameter("order").as_int();
    goal_msg.target_value = this->get_parameter("target_value").as_double();
    
    RCLCPP_INFO(this->get_logger(), "发送目标: order=%d, target_value=%f",
               goal_msg.order, goal_msg.target_value);
    
    // 定义反馈回调
    auto send_goal_options = rclcpp_action::Client<CustomAction>::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&ProgressActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
    // 定义目标响应回调
    send_goal_options.goal_response_callback =
      std::bind(&ProgressActionClient::goal_response_callback, this, std::placeholders::_1);
    
    // 定义结果回调
    send_goal_options.result_callback =
      std::bind(&ProgressActionClient::result_callback, this, std::placeholders::_1);
    
    // 发送目标
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<CustomAction>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  void goal_response_callback(const GoalHandleCustomAction::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "目标被接受");
  }
  
  void feedback_callback(
    GoalHandleCustomAction::SharedPtr,
    const std::shared_ptr<const CustomAction::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "收到反馈: 当前值 = %f, 完成度 = %.1f%%",
               feedback->current_value, feedback->completion_percentage);
  }
  
  void result_callback(const GoalHandleCustomAction::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "目标成功完成");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "目标被中止");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "目标被取消");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "未知的结果代码");
        break;
    }
    
    RCLCPP_INFO(this->get_logger(), "结果: success=%s, final_value=%f, message='%s'",
               result.result->success ? "true" : "false",
               result.result->final_value,
               result.result->message.c_str());
               
    // 如果是单次运行模式，退出
    if (!this->get_parameter("auto_send").as_bool()) {
      rclcpp::shutdown();
    } else {
      // 否则，创建新的定时器以发送下一个目标
      this->timer_ = this->create_wall_timer(
        5s, std::bind(&ProgressActionClient::send_goal, this));
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgressActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 