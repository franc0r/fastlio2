import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )

    use_sim_time = launch.substitutions.LaunchConfiguration("use_sim_time")

    declare_use_sim_time = launch.actions.DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    return launch.LaunchDescription(
        [
            declare_use_sim_time,
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                # prefix=['gdbserver localhost:3000'],
                parameters=[
                    {"config_path": config_path.perform(launch.LaunchContext())},
                    {"use_sim_time": use_sim_time},
                ]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="fastlio2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
                parameters=[
                    {"use_sim_time": use_sim_time},
                ]
            ),
        ]
    )
