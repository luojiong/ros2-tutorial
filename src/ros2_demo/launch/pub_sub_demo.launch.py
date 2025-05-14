from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """生成发布者和订阅者的启动描述"""
    
    # 声明启动参数
    prefix_launch_arg = DeclareLaunchArgument(
        'prefix',
        default_value='你好',
        description='发布者消息的前缀'
    )
    
    filter_launch_arg = DeclareLaunchArgument(
        'filter_even',
        default_value='False',
        description='是否过滤偶数ID的消息'
    )
    
    # 配置节点
    publisher_node = Node(
        package='ros2_demo',
        executable='publisher',
        name='demo_publisher',
        parameters=[{
            'message_prefix': LaunchConfiguration('prefix'),
        }],
        output='screen'
    )
    
    subscriber_node = Node(
        package='ros2_demo',
        executable='subscriber',
        name='demo_subscriber',
        parameters=[{
            'filter_even_ids': LaunchConfiguration('filter_even'),
        }],
        output='screen'
    )
    
    # 创建并返回启动描述
    return LaunchDescription([
        prefix_launch_arg,
        filter_launch_arg,
        publisher_node,
        subscriber_node
    ]) 