import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 该文件仅用于测试世界模型在gazebo中显示正不正常

model_path = LaunchConfiguration('model')
world_path = LaunchConfiguration('world')
robot_name_in_model = "robot"

def declare_parameters():
    # 声明参数
    model_path_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory('robot_sim_ros2_control'),'urdf','robot_sim.xacro'),
        description='URDF 的绝对路径'
    )

    world_path_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(get_package_share_directory('robot_sim_bringup'),'world','rmul_2025','rmul_2025_world.sdf'),
        description='world.sdf文件 的绝对路径'
    )

    return [model_path_arg,world_path_arg]

def gazebo():
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': world_path,
        }.items(),
    )
    return [gazebo_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    gazebo_node = gazebo()
    
    return LaunchDescription(
        declare_parameters_node +
        gazebo_node
    )
