from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package="lifecycle_demo", 
            executable="lifecycle_talker",
            name="lifecycle_talker",
            namespace="cpp",
            output="screen"
        ),
        Node(
            package="lifecycle_demo",
            executable="lifecycle_listener",
            name="lifecycle_listener",
            namespace="cpp",
            output="screen"
        ),
        Node(
            package="lifecycle_demo",
            executable="lifecycle_service_client",
            name="lifecycle_service_client",
            namespace="cpp",
            output="screen",
            on_exit=Shutdown()
        )
    ])