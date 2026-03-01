import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage

use_sim_time = LaunchConfiguration('use_sim_time')
map_path = LaunchConfiguration('map_path')
nav2_params_file = LaunchConfiguration('nav2_params_file')
slam_params_file = LaunchConfiguration('slam_params_file')

def declare_parameters():
    # 声明新的 Launch 参数
    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='如果为 true 则使用仿真时钟(Gazebo)'
    )
    declare_map_path_arg = DeclareLaunchArgument(
        name='map_path',
        default_value=PathJoinSubstitution([]),
        description='地图文件的绝对路径'
    )
    # declare_nav2_param_path_arg = DeclareLaunchArgument(
    #     name='nav2_params_file',
    #     default_value=os.path.join(get_package_share_directory('robot_navigation2'), 'maps', 'room.yaml'),
    #     description='Nav2 参数文件的绝对路径'
    # )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        name='slam_params_file',
        default_value=os.path.join(get_package_share_directory("slam_toolbox"),'config', 'mapper_params_online_async.yaml'),
        description='slam_toolbox 节点使用的 ROS2 参数文件的完整路径'
    )
    return [declare_use_sim_time_arg,declare_map_path_arg,declare_slam_params_file_cmd]

def slam_toolbox():
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')]),
        launch_arguments={
            'autostart': 'true',                            # 是否自动启动 slam_toolbox。如果 use_lifecycle_manager 为 true，则忽略该参数。
            'use_lifecycle_manager':'false',                # 启用生命周期管理器节点激活时的 bond 连接
            'use_sim_time':use_sim_time,
            'slam_params_file':slam_params_file             # slam_toolbox 节点使用的 ROS2 参数文件的完整路径
        }.items()
    )
    return [slam_toolbox_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    slam_toolbox_node = slam_toolbox()

    return LaunchDescription(
        declare_parameters_node +
        slam_toolbox_node
    )
