#include <memory>
#include <thread>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_demo/action/custom_action.hpp"

class ProgressActionServer : public rclcpp::Node
{
public:
  using CustomAction = ros2_demo::action::CustomAction;
  using GoalHandleCustomAction = rclcpp_action::ServerGoalHandle<CustomAction>;
  
  explicit ProgressActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("progress_action_server", options)
  {
    using namespace std::placeholders;
    
    this->action_server_ = rclcpp_action::create_server<CustomAction>(
      this,
      "progress_tracker",
      std::bind(&ProgressActionServer::handle_goal, this, _1, _2),
      std::bind(&ProgressActionServer::handle_cancel, this, _1),
      std::bind(&ProgressActionServer::handle_accepted, this, _1));
      
    RCLCPP_INFO(this->get_logger(), "进度跟踪Action服务器已启动");
  }

private:
  rclcpp_action::Server<CustomAction>::SharedPtr action_server_;
  
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CustomAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "收到目标: order=%d, target_value=%f",
               goal->order, goal->target_value);
    
    // 简单验证
    if (goal->target_value <= 0) {
      RCLCPP_WARN(this->get_logger(), "拒绝目标: 目标值必须为正数");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCustomAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消请求");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void handle_accepted(const std::shared_ptr<GoalHandleCustomAction> goal_handle)
  {
    // 在单独的线程中执行，以免阻塞
    std::thread{std::bind(&ProgressActionServer::execute, this, goal_handle)}.detach();
  }
  
  void execute(const std::shared_ptr<GoalHandleCustomAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "执行目标");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CustomAction::Feedback>();
    auto result = std::make_shared<CustomAction::Result>();
    
    // 根据订单决定递增步长
    double step = 0.1;
    if (goal->order > 1) {
      step = 0.05 * goal->order;
    }
    
    // 初始化当前值
    feedback->current_value = 0.0;
    
    // 设置初始进度
    feedback->completion_percentage = 0.0;
    
    // 进度更新，每100ms一次
    rclcpp::Rate loop_rate(10);
    
    while (rclcpp::ok() && feedback->current_value < goal->target_value) {
      // 检查是否被取消
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->final_value = feedback->current_value;
        result->message = "目标已取消";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "目标已取消");
        return;
      }
      
      // 更新进度
      feedback->current_value += step;
      
      // 确保不超过目标值
      if (feedback->current_value > goal->target_value) {
        feedback->current_value = goal->target_value;
      }
      
      // 计算进度百分比
      feedback->completion_percentage = 
        100.0 * feedback->current_value / goal->target_value;
      
      // 发送反馈
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "发布反馈: %.1f%% 完成", 
                 feedback->completion_percentage);
      
      loop_rate.sleep();
    }
    
    // 检查目标是否完成
    if (rclcpp::ok() && !goal_handle->is_canceling()) {
      // 设置最终结果
      result->success = true;
      result->final_value = goal->target_value;
      result->message = "目标已完成";
      
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "目标已完成");
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProgressActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 