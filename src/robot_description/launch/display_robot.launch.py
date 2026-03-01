import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def declare_parameters():
    # 获取默认路径
    robot_description_path = get_package_share_directory('robot_description')

    # 声明参数
    model_path_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(robot_description_path,'urdf','robot','robot.urdf.xacro'),
        description='URDF 的绝对路径'
    )

    rviz_path_arg = DeclareLaunchArgument(
        name='rviz',
        default_value=os.path.join(robot_description_path,'config','rviz','display_model.rviz'),
        description='URDF 的绝对路径'
    )

    return [model_path_arg,rviz_path_arg]

def robot_description():
    # 获取文件内容生成新的参数
    robot_description = ParameterValue(launch.substitutions.Command(['xacro ', LaunchConfiguration('model')]),value_type=str)

    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    # RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz')]
    )

    return [robot_state_publisher_node,joint_state_publisher_node,rviz_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    robot_description_node = robot_description()
    return LaunchDescription(
        declare_parameters_node +
        robot_description_node
    )
