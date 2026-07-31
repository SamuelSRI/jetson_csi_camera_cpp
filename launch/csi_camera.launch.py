from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("jetson_csi_camera"))
    default_params = package_share / "config" / "camera.yaml"

    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    node_name = LaunchConfiguration("node_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=str(default_params),
            description="Path to the camera parameter file",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="camera/front",
            description="ROS namespace",
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="jetson_csi_camera",
            description="ROS node name",
        ),
        Node(
            package="jetson_csi_camera",
            executable="csi_camera_node",
            name=node_name,
            namespace=namespace,
            output="screen",
            parameters=[params_file],
            emulate_tty=True,
        ),
    ])
