from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("railbreak_backup_odometry"), "config", "params.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("assets_dir", default_value=""),
        DeclareLaunchArgument("output_frame", default_value="mgrs"),
        Node(
            package="railbreak_backup_odometry",
            executable="backup_odometry_node",
            name="backup_odometry",
            output="screen",
            parameters=[
                params,
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "assets_dir": LaunchConfiguration("assets_dir"),
                    "output_frame": LaunchConfiguration("output_frame"),
                },
            ],
        ),
    ])
