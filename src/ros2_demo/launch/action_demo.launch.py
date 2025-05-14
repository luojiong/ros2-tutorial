from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """生成Action服务器和客户端的启动描述"""
    
    # 声明启动参数
    order_arg = DeclareLaunchArgument(
        'order',
        default_value='3',
        description='订单值，影响进度步长'
    )
    
    target_arg = DeclareLaunchArgument(
        'target_value',
        default_value='20.0',
        description='目标值'
    )
    
    auto_send_arg = DeclareLaunchArgument(
        'auto_send',
        default_value='true',
        description='是否自动发送目标'
    )
    
    # 配置节点
    action_server = Node(
        package='ros2_demo',
        executable='action_server',
        name='progress_server',
        output='screen'
    )
    
    action_client = Node(
        package='ros2_demo',
        executable='action_client',
        name='progress_client',
        parameters=[{
            'order': LaunchConfiguration('order'),
            'target_value': LaunchConfiguration('target_value'),
            'auto_send': LaunchConfiguration('auto_send'),
        }],
        output='screen'
    )
    
    # 创建并返回启动描述
    return LaunchDescription([
        order_arg,
        target_arg,
        auto_send_arg,
        action_server,
        action_client
    ]) 