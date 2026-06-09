from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    config_file = LaunchConfiguration("config_file")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="trunk_robot",
            description="ROS namespace for the trunk robot.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("trunk_gravity_compensation"),
                "config",
                "gravity_compensation.yaml",
            ]),
            description="Gravity compensation observer parameter file.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level for the gravity compensation node.",
        ),
        Node(
            package="trunk_gravity_compensation",
            executable="gravity_compensation_node",
            name="gravity_compensation_node",
            namespace=namespace,
            parameters=[config_file],
            arguments=["--ros-args", "--log-level", log_level],
            output="screen",
        ),
    ])
