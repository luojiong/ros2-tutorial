# ROS2 DEMO

这是一个全面的ROS2 C++演示项目，展示了ROS2的常用功能。

## 功能特性

本演示包含以下ROS2功能：

- 话题通信（发布者/订阅者）
  - 标准消息和自定义消息
  - 参数配置
  
- 服务通信（服务端/客户端）
  - 同步和异步调用
  - 错误处理
  
- 动作（Action）通信
  - 带进度反馈的长时间运行任务
  - 目标取消
  
- 参数系统
  - 各种类型的参数（整数、浮点数、字符串、布尔值、数组）
  - 参数验证
  - 参数动态更新
  
- 启动系统
  - 单独启动各个演示
  - 组合启动所有演示

## 自定义接口

本项目定义了以下自定义接口：

- **消息(Message)**: `CustomMessage.msg`
- **服务(Service)**: `CustomService.srv`
- **动作(Action)**: `CustomAction.action`

## 编译与运行

### 环境要求

- ROS2（建议使用Foxy、Galactic、Humble或更新版本）
- C++17兼容的编译器

### 编译步骤

1. 将本仓库复制到ROS2工作空间的`src`目录：

```bash
# 创建工作空间（如果没有的话）
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 复制或克隆本仓库
# git clone <仓库URL>

# 返回工作空间根目录
cd ~/ros2_ws

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install
```

2. 在每个新的终端中，记得先加载工作空间：

```bash
source ~/ros2_ws/install/setup.bash
```

### 运行演示

#### 1. 话题演示（发布者/订阅者）

```bash
ros2 launch ros2_demo pub_sub_demo.launch.py
```

参数：
- `prefix`: 消息前缀，默认为"你好"
- `filter_even`: 是否过滤偶数ID的消息，默认为"false"

#### 2. 服务演示（服务端/客户端）

```bash
ros2 launch ros2_demo service_demo.launch.py
```

参数：
- `operation`: 计算操作，可选"add"、"subtract"、"multiply"、"divide"
- `a`: 第一个操作数
- `b`: 第二个操作数

#### 3. 动作演示（Action服务器/客户端）

```bash
ros2 launch ros2_demo action_demo.launch.py
```

参数：
- `order`: 订单值，影响进度步长
- `target_value`: 目标值
- `auto_send`: 是否自动发送目标

#### 4. 参数演示

```bash
ros2 launch ros2_demo param_demo.launch.py
```

参数：
- `string_param`: 字符串参数示例
- `int_param`: 整数参数示例
- `double_param`: 浮点数参数示例
- `bool_param`: 布尔参数示例

#### 5. 全部功能演示

```bash
ros2 launch ros2_demo full_demo.launch.py
```

## 学习资源

- [ROS2 文档](https://docs.ros.org/en/humble/index.html)
- [ROS2 教程](https://docs.ros.org/en/humble/Tutorials.html)
