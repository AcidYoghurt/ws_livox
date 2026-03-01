import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

model_path = LaunchConfiguration('model')
rviz_path = LaunchConfiguration('rviz')
world_path = LaunchConfiguration('world')
use_sim_time = LaunchConfiguration('use_sim_time')

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

    world_path_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(get_package_share_directory('robot_sim_bringup'),'world','rmul_2025','rmul_2025_world.sdf'),
        description='world.sdf文件 的绝对路径'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='是否开启仿真时间'
    )

    return [model_path_arg,rviz_path_arg,world_path_arg,use_sim_time_arg]

def robot_description():
    # 获取文件内容生成新的参数
    robot_description = ParameterValue(Command(['xacro ', model_path]),value_type=str,)

    # 发布robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 启动 RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return [robot_state_publisher_node,rviz_node]

def ros2_control(spawn_node):
    # 加载控制器
    # 使用 spawner 节点，它会等待 controller_manager 服务可用
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot_joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "30",
            "--service-call-timeout", "30"
        ],
        output="screen",
    )

    # 加载差速驱动控制器（差速控制器与力控制器本质上是一样的，所以两个启动一个就够了）
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot_diff_drive_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "30",
            "--service-call-timeout", "30"
        ],
        output="screen",
    )

    # 等机器人实体创建完成后，再开始加载控制器，避免仿真初始化阶段的激活超时
    start_joint_state_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # joint_state_broadcaster 完成后再启动 diff_drive_controller，减少切换竞争
    start_diff_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return [start_joint_state_after_spawn, start_diff_after_joint_state]

def gazebo():
    # 启动gazebo服务和GUI
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': world_path
        }.items(),
    )

    # 把机器人生成到 Gazebo
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-topic', '/robot_description',
            '-allow_renaming', 'true',
            '-z', '0.2'
        ],
        output='screen',
    )

    # ros2到gazebo的桥
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[

            # clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # camera
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

            # imu
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',

            # 雷达
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',

            # diff drive由ros2_control在ROS侧处理，这里不桥接cmd_vel/odom，避免同名话题双来源

        ],
        output='screen'
    )
    return [gazebo_node, spawn_node, bridge], spawn_node

# 键盘控制节点
def teleop_twist_keyboard():
    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --', # 新开终端
        arguments=[
            '--ros-args',
            '-p', 'stamped:=true',                    # 如果为 false（默认值），则发布geometry_msgs/msg/Twist消息。如果为 true，则发布geometry_msgs/msg/TwistStamped消息。
            '-p', 'frame_id:=base_footprint',         # 当stamped为真时，表示发布消息时要使用的 frame_id（因为此时多加了一个Header）
            '-p', 'speed:=0.5',                       # 节点默认的启动速度。
            '-p', 'turn:=1.0'                         # 节点默认的初始转向率（弧度/秒）
        ]
    )
    return [teleop_twist_keyboard_node]

def generate_launch_description():
    declare_parameters_node = declare_parameters()
    robot_description_node = robot_description()
    gazebo_node, spawn_node = gazebo()
    teleop_twist_keyboard_node = teleop_twist_keyboard()
    ros2_control_node = ros2_control(spawn_node)

    return LaunchDescription(
        declare_parameters_node +
        gazebo_node +
        robot_description_node +
        teleop_twist_keyboard_node +
        ros2_control_node
    )
