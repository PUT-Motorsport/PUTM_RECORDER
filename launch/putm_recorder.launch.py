import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('putm_recorder'),
        'config',
        'putm_recorder.yaml'
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                namespace="putm_recorder",
                package="putm_recorder",
                executable="rec_node",
                name="rec_node",
                parameters=[config],
            ),
        ]
    )
