#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_demo/msg/custom_message.hpp"

using namespace std::chrono_literals;

class DemoPublisher : public rclcpp::Node
{
public:
  DemoPublisher()
  : Node("demo_publisher"), count_(0)
  {
    // 创建标准字符串消息的发布者
    publisher_ = this->create_publisher<std_msgs::msg::String>("demo_topic", 10);
    
    // 创建自定义消息的发布者
    custom_publisher_ = this->create_publisher<ros2_demo::msg::CustomMessage>("custom_topic", 10);
    
    // 创建定时器，每1秒发布一次消息
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&DemoPublisher::timer_callback, this));

    // 声明参数
    this->declare_parameter("message_prefix", "Hello");
  }

private:
  void timer_callback()
  {
    // 从参数获取消息前缀
    std::string prefix = this->get_parameter("message_prefix").as_string();
    
    // 发布标准消息
    auto standard_message = std_msgs::msg::String();
    standard_message.data = prefix + " ROS2世界! 计数: " + std::to_string(count_);
    RCLCPP_INFO(this->get_logger(), "发布标准消息: '%s'", standard_message.data.c_str());
    publisher_->publish(standard_message);
    
    // 发布自定义消息
    auto custom_message = ros2_demo::msg::CustomMessage();
    custom_message.id = count_;
    custom_message.message = "这是自定义消息 #" + std::to_string(count_);
    custom_message.value = 1.0 * count_;
    custom_message.flag = (count_ % 2 == 0);
    RCLCPP_INFO(this->get_logger(), "发布自定义消息 ID: %d, Value: %f", 
               custom_message.id, custom_message.value);
    custom_publisher_->publish(custom_message);
    
    count_++;
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<ros2_demo::msg::CustomMessage>::SharedPtr custom_publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 