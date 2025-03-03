from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 启动三维建图节点
        Node(
            package='PointCloud_Registration',  # 三维建图功能包名称
            executable='PointCloud_Registration',  # 三维建图节点名称
            name='PointCloud_Registration',  # 节点命名
            output='screen',
        )
    ])
