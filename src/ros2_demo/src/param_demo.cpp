#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamDemoNode : public rclcpp::Node
{
public:
  ParamDemoNode()
  : Node("param_demo")
  {
    // 声明和初始化各种类型的参数
    this->declare_parameter("string_param", "默认字符串");
    this->declare_parameter("int_param", 42);
    this->declare_parameter("double_param", 3.14);
    this->declare_parameter("bool_param", true);
    
    // 数组参数
    std::vector<int> default_int_array = {1, 2, 3, 4, 5};
    this->declare_parameter("int_array_param", default_int_array);
    
    std::vector<double> default_double_array = {1.1, 2.2, 3.3};
    this->declare_parameter("double_array_param", default_double_array);
    
    std::vector<std::string> default_string_array = {"one", "two", "three"};
    this->declare_parameter("string_array_param", default_string_array);
    
    // 创建定时器，定期打印参数值
    timer_ = this->create_wall_timer(
      2000ms, std::bind(&ParamDemoNode::timer_callback, this));
      
    // 创建参数变化回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ParamDemoNode::parameters_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "参数演示节点已启动");
    RCLCPP_INFO(this->get_logger(), "可以使用命令行修改参数，例如:");
    RCLCPP_INFO(this->get_logger(), "ros2 param set /param_demo string_param 新的字符串值");
  }

private:
  void timer_callback()
  {
    // 获取并打印所有参数
    RCLCPP_INFO(this->get_logger(), "当前参数值:");
    RCLCPP_INFO(this->get_logger(), "-----------------------");
    
    // 获取基本参数
    auto string_param = this->get_parameter("string_param").as_string();
    auto int_param = this->get_parameter("int_param").as_int();
    auto double_param = this->get_parameter("double_param").as_double();
    auto bool_param = this->get_parameter("bool_param").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "string_param: %s", string_param.c_str());
    RCLCPP_INFO(this->get_logger(), "int_param: %d", int_param);
    RCLCPP_INFO(this->get_logger(), "double_param: %f", double_param);
    RCLCPP_INFO(this->get_logger(), "bool_param: %s", bool_param ? "true" : "false");
    
    // 获取数组参数并打印
    try {
      auto int_array = this->get_parameter("int_array_param").as_integer_array();
      std::string int_array_str = "int_array_param: [";
      for (auto value : int_array) {
        int_array_str += std::to_string(value) + ", ";
      }
      int_array_str += "]";
      RCLCPP_INFO(this->get_logger(), "%s", int_array_str.c_str());
      
      auto double_array = this->get_parameter("double_array_param").as_double_array();
      std::string double_array_str = "double_array_param: [";
      for (auto value : double_array) {
        double_array_str += std::to_string(value) + ", ";
      }
      double_array_str += "]";
      RCLCPP_INFO(this->get_logger(), "%s", double_array_str.c_str());
      
      auto string_array = this->get_parameter("string_array_param").as_string_array();
      std::string string_array_str = "string_array_param: [";
      for (auto value : string_array) {
        string_array_str += value + ", ";
      }
      string_array_str += "]";
      RCLCPP_INFO(this->get_logger(), "%s", string_array_str.c_str());
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
      RCLCPP_ERROR(this->get_logger(), "参数访问错误: %s", e.what());
    }
    
    RCLCPP_INFO(this->get_logger(), "-----------------------");
  }
  
  // 参数变化回调
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "成功";
    
    for (const auto & param : parameters) {
      RCLCPP_INFO(this->get_logger(), "参数变化: %s", param.get_name().c_str());
      
      // 添加自定义验证逻辑
      if (param.get_name() == "int_param") {
        if (param.as_int() < 0) {
          result.successful = false;
          result.reason = "int_param 必须为非负数";
          break;
        }
      }
      
      if (param.get_name() == "double_param") {
        if (param.as_double() < 0.0 || param.as_double() > 10.0) {
          result.successful = false;
          result.reason = "double_param 必须在 0.0 到 10.0 之间";
          break;
        }
      }
    }
    
    return result;
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParamDemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 