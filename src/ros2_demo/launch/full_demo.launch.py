from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """生成完整演示的启动描述，包含所有演示节点"""
    
    # 获取包的共享目录
    ros2_demo_share_dir = FindPackageShare('ros2_demo')
    
    # 声明通用参数
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='全面演示',
        description='消息前缀'
    )
    
    # 包含其他启动文件
    pub_sub_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ros2_demo_share_dir, 'launch', 'pub_sub_demo.launch.py'])
        ]),
        launch_arguments={
            'prefix': LaunchConfiguration('prefix'),
            'filter_even': 'true'
        }.items()
    )
    
    service_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ros2_demo_share_dir, 'launch', 'service_demo.launch.py'])
        ]),
        launch_arguments={
            'operation': 'add',
            'a': '100',
            'b': '50'
        }.items()
    )
    
    action_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ros2_demo_share_dir, 'launch', 'action_demo.launch.py'])
        ]),
        launch_arguments={
            'order': '5',
            'target_value': '30.0',
            'auto_send': 'true'
        }.items()
    )
    
    param_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ros2_demo_share_dir, 'launch', 'param_demo.launch.py'])
        ]),
        launch_arguments={
            'string_param': '完整演示中的参数',
            'int_param': '42',
            'double_param': '3.14',
            'bool_param': 'true'
        }.items()
    )
    
    # 创建并返回启动描述
    return LaunchDescription([
        prefix_arg,
        pub_sub_demo,
        service_demo,
        action_demo,
        param_demo
    ])