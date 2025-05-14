#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_demo/msg/custom_message.hpp"

using std::placeholders::_1;

class DemoSubscriber : public rclcpp::Node
{
public:
  DemoSubscriber()
  : Node("demo_subscriber")
  {
    // 创建标准消息的订阅者
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "demo_topic", 10, std::bind(&DemoSubscriber::topic_callback, this, _1));
    
    // 创建自定义消息的订阅者
    custom_subscription_ = this->create_subscription<ros2_demo::msg::CustomMessage>(
      "custom_topic", 10, std::bind(&DemoSubscriber::custom_topic_callback, this, _1));
    
    // 声明参数
    this->declare_parameter("filter_even_ids", false);
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "收到标准消息: '%s'", msg->data.c_str());
  }
  
  void custom_topic_callback(const ros2_demo::msg::CustomMessage::SharedPtr msg)
  {
    // 获取参数值
    bool filter_even_ids = this->get_parameter("filter_even_ids").as_bool();
    
    // 如果启用了过滤并且ID是偶数，则跳过消息处理
    if (filter_even_ids && msg->id % 2 == 0) {
      RCLCPP_INFO(this->get_logger(), "过滤掉偶数ID的消息: %d", msg->id);
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "收到自定义消息:");
    RCLCPP_INFO(this->get_logger(), "  ID: %d", msg->id);
    RCLCPP_INFO(this->get_logger(), "  消息: %s", msg->message.c_str());
    RCLCPP_INFO(this->get_logger(), "  数值: %f", msg->value);
    RCLCPP_INFO(this->get_logger(), "  标志: %s", msg->flag ? "true" : "false");
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<ros2_demo::msg::CustomMessage>::SharedPtr custom_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 