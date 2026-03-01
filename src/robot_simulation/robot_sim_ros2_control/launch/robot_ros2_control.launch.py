import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

model_path = LaunchConfiguration('model')
rviz_path = LaunchConfiguration('rviz')

def declare_parameters():
    # 声明参数
    model_path_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory('robot_sim_ros2_control'),'urdf','robot_sim.xacro'),
        description='URDF 的绝对路径'
    )

    rviz_path_arg = DeclareLaunchArgument(
        name='rviz',
        default_value=os.path.join(get_package_share_directory('robot_description'),'config','rviz','display_model.rviz'),
        description='URDF 的绝对路径'
    )

    return [model_path_arg,rviz_path_arg]

def robot_description():
    # 获取文件内容生成新的参数
    robot_description = ParameterValue(Command(['xacro ', model_path]),value_type=str)

    # 发布robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 启动 RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_path]
    )

    return [robot_state_publisher_node,rviz_node]

def ros2_control():
    # 加载控制器
    # 使用 spawner 节点，它会等待 controller_manager 服务可用
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_effort_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 加载差速驱动控制器（差速控制器与力控制器本质上是一样的，所以两个启动一个就够了）
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return [joint_state_broadcaster_spawner, diff_drive_spawner]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    robot_description_node = robot_description()
    ros2_control_node = ros2_control()

    return LaunchDescription(
        declare_parameters_node +
        robot_description_node +
        ros2_control_node
    )
