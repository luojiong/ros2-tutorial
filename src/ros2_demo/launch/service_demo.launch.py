from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """生成服务器和客户端的启动描述"""
    
    # 声明启动参数
    operation_arg = DeclareLaunchArgument(
        'operation',
        default_value='add',
        description='计算操作（add, subtract, multiply, divide）'
    )
    
    a_arg = DeclareLaunchArgument(
        'a',
        default_value='10',
        description='操作数 a'
    )
    
    b_arg = DeclareLaunchArgument(
        'b',
        default_value='5',
        description='操作数 b'
    )
    
    # 配置节点
    server_node = Node(
        package='ros2_demo',
        executable='service_server',
        name='calculator_server',
        output='screen'
    )
    
    client_node = Node(
        package='ros2_demo',
        executable='service_client',
        name='calculator_client',
        parameters=[{
            'operation': LaunchConfiguration('operation'),
            'a': LaunchConfiguration('a'),
            'b': LaunchConfiguration('b'),
        }],
        output='screen'
    )
    
    # 创建并返回启动描述
    return LaunchDescription([
        operation_arg,
        a_arg,
        b_arg,
        server_node,
        client_node
    ]) 