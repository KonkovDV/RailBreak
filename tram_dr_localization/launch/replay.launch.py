from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("tram_dr_localization")
    estimator_yaml = PathJoinSubstitution([pkg, "config", "estimator.yaml"])
    vehicle_yaml = PathJoinSubstitution([pkg, "config", "vehicle_lvenok_moscow.yaml"])
    route_yaml = PathJoinSubstitution([pkg, "config", "route_10.yaml"])
    customer_yaml = PathJoinSubstitution([pkg, "config", "customer_topics.yaml"])
    bag = LaunchConfiguration("bag")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim = {"use_sim_time": use_sim_time}
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag",
                default_value="",
                description="rosbag2 directory to play; empty = no player",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="true for bag replay with --clock; live.launch sets false",
            ),
            DeclareLaunchArgument(
                "vehicle",
                default_value=vehicle_yaml,
                description="vehicle YAML: lvenok_moscow (route 10 default), combino_nf100 twin, vityaz_m",
            ),
            Node(
                package="tram_dr_localization",
                executable="topic_adapter_node",
                name="topic_adapter",
                parameters=[customer_yaml, sim, LaunchConfiguration("vehicle")],
                output="screen",
            ),
            Node(
                package="tram_dr_localization",
                executable="state_estimator_node",
                name="state_estimator",
                parameters=[estimator_yaml, customer_yaml, route_yaml, sim,
                            LaunchConfiguration("vehicle")],
                output="screen",
            ),
            Node(
                package="tram_dr_localization",
                executable="fault_monitor_node",
                name="fault_monitor",
                parameters=[sim],
                output="screen",
            ),
            Node(
                package="tram_dr_localization",
                executable="map_projector_node",
                name="map_projector",
                parameters=[route_yaml, sim],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag, "--clock"],
                condition=IfCondition(PythonExpression(["'", bag, "' != ''"])),
                output="screen",
            ),
        ]
    )
