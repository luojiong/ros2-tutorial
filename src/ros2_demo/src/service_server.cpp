#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_demo/srv/custom_service.hpp"

class CalculatorServiceServer : public rclcpp::Node
{
public:
  CalculatorServiceServer()
  : Node("calculator_service_server")
  {
    // 创建服务
    service_ = this->create_service<ros2_demo::srv::CustomService>(
      "calculator_service",
      std::bind(&CalculatorServiceServer::handle_service, this,
                std::placeholders::_1, std::placeholders::_2));
                
    RCLCPP_INFO(this->get_logger(), "计算器服务已启动");
  }

private:
  void handle_service(
    const std::shared_ptr<ros2_demo::srv::CustomService::Request> request,
    std::shared_ptr<ros2_demo::srv::CustomService::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "收到请求: a=%d, b=%d, 操作='%s'",
               request->a, request->b, request->operation.c_str());
    
    response->success = true;
    response->message = "计算成功";
    
    if (request->operation == "add") {
      response->result = static_cast<double>(request->a + request->b);
    } else if (request->operation == "subtract") {
      response->result = static_cast<double>(request->a - request->b);
    } else if (request->operation == "multiply") {
      response->result = static_cast<double>(request->a * request->b);
    } else if (request->operation == "divide") {
      if (request->b == 0) {
        response->success = false;
        response->message = "除数不能为零";
        response->result = 0.0;
      } else {
        response->result = static_cast<double>(request->a) / request->b;
      }
    } else {
      response->success = false;
      response->message = "不支持的操作: " + request->operation;
      response->result = 0.0;
    }
    
    RCLCPP_INFO(this->get_logger(), "发送响应: result=%f, success=%s, message='%s'",
               response->result, response->success ? "true" : "false", 
               response->message.c_str());
  }
  
  rclcpp::Service<ros2_demo::srv::CustomService>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CalculatorServiceServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 