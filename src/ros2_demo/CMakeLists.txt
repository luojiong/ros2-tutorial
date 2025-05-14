cmake_minimum_required(VERSION 3.8)
project(ros2_demo)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# 生成自定义消息、服务和动作接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
  "srv/CustomService.srv"
  "action/CustomAction.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# 包含目录
include_directories(include)

# 添加可执行文件

# 1. 发布者
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)
rosidl_target_interfaces(publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 2. 订阅者
add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)
rosidl_target_interfaces(subscriber ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 3. 服务端
add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp example_interfaces)
rosidl_target_interfaces(service_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 4. 客户端
add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp example_interfaces)
rosidl_target_interfaces(service_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 5. Action服务器
add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action action_msgs)
rosidl_target_interfaces(action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 6. Action客户端
add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action action_msgs)
rosidl_target_interfaces(action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 7. 参数演示
add_executable(param_demo src/param_demo.cpp)
ament_target_dependencies(param_demo rclcpp)

# 安装可执行文件
install(TARGETS
  publisher
  subscriber
  service_server
  service_client
  action_server
  action_client
  param_demo
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 