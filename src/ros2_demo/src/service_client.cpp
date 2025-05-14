#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "ros2_demo/srv/custom_service.hpp"

using namespace std::chrono_literals;

class CalculatorServiceClient : public rclcpp::Node
{
public:
  CalculatorServiceClient()
  : Node("calculator_service_client")
  {
    // 创建客户端
    client_ = this->create_client<ros2_demo::srv::CustomService>("calculator_service");
    
    // 创建定时器，如果没有在启动时指定参数，则定期发送请求
    if (!this->has_parameter("operation")) {
      this->declare_parameter("operation", "add");
      this->declare_parameter("a", 5);
      this->declare_parameter("b", 3);
      
      // 每3秒发送一个请求
      timer_ = this->create_wall_timer(
        3000ms, std::bind(&CalculatorServiceClient::send_request, this));
    } else {
      // 立即发送一个请求
      send_request();
    }
  }

  void send_request()
  {
    // 等待服务可用
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "中断等待服务。退出。");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待计算器服务...");
    }
    
    // 获取参数
    std::string operation = this->get_parameter("operation").as_string();
    int a = this->get_parameter("a").as_int();
    int b = this->get_parameter("b").as_int();
    
    // 创建请求
    auto request = std::make_shared<ros2_demo::srv::CustomService::Request>();
    request->a = a;
    request->b = b;
    request->operation = operation;
    
    RCLCPP_INFO(this->get_logger(), "发送请求: a=%d, b=%d, 操作='%s'",
               request->a, request->b, request->operation.c_str());
    
    // 异步发送请求
    auto future = client_->async_send_request(
      request, std::bind(&CalculatorServiceClient::response_callback, this, std::placeholders::_1));
  }

private:
  void response_callback(rclcpp::Client<ros2_demo::srv::CustomService>::SharedFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "收到响应: result=%f, success=%s, message='%s'",
               response->result, response->success ? "true" : "false", 
               response->message.c_str());
    
    // 如果我们是从命令行参数启动的，完成服务调用后退出
    if (!timer_) {
      RCLCPP_INFO(this->get_logger(), "服务请求完成，退出。");
      rclcpp::shutdown();
    } else {
      // 否则继续循环，尝试不同的操作
      static const std::vector<std::string> operations = {"add", "subtract", "multiply", "divide"};
      static size_t op_index = 0;
      
      op_index = (op_index + 1) % operations.size();
      this->set_parameter(rclcpp::Parameter("operation", operations[op_index]));
      
      // 随机更改数值
      this->set_parameter(rclcpp::Parameter("a", this->get_parameter("a").as_int() + 1));
      
      if (operations[op_index] == "divide") {
        // 确保除数不为零
        this->set_parameter(rclcpp::Parameter("b", 2));
      } else {
        this->set_parameter(rclcpp::Parameter("b", this->get_parameter("b").as_int() - 1));
      }
    }
  }
  
  rclcpp::Client<ros2_demo::srv::CustomService>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CalculatorServiceClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 