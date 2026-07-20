import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                namespace="putm_recorder",
                package="putm_recorder",
                executable="rec_node",
                name="rec_node",
            ),
        ]
    )
