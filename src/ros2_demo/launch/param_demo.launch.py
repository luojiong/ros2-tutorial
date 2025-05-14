from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """生成参数演示的启动描述"""
    
    # 声明启动参数
    string_arg = DeclareLaunchArgument(
        'string_param',
        default_value='从启动文件设置的字符串',
        description='字符串参数示例'
    )
    
    int_arg = DeclareLaunchArgument(
        'int_param',
        default_value='100',
        description='整数参数示例'
    )
    
    double_arg = DeclareLaunchArgument(
        'double_param',
        default_value='3.1415926',
        description='浮点数参数示例'
    )
    
    bool_arg = DeclareLaunchArgument(
        'bool_param',
        default_value='false',
        description='布尔参数示例'
    )
    
    # 配置节点
    param_node = Node(
        package='ros2_demo',
        executable='param_demo',
        name='param_demo',
        parameters=[{
            'string_param': LaunchConfiguration('string_param'),
            'int_param': LaunchConfiguration('int_param'),
            'double_param': LaunchConfiguration('double_param'),
            'bool_param': LaunchConfiguration('bool_param'),
            'int_array_param': [10, 20, 30, 40, 50],
            'double_array_param': [1.1, 2.2, 3.3, 4.4],
            'string_array_param': ['启动', '文件', '数组', '示例'],
        }],
        output='screen'
    )
    
    # 创建并返回启动描述
    return LaunchDescription([
        string_arg,
        int_arg,
        double_arg,
        bool_arg,
        param_node
    ]) 