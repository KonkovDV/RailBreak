from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Same nodes as replay without a bag player. Wall-clock, not /clock.
def generate_launch_description():
    replay = PathJoinSubstitution(
        [FindPackageShare("tram_dr_localization"), "launch", "replay.launch.py"]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(replay),
                launch_arguments={"use_sim_time": "false"}.items(),
            )
        ]
    )
