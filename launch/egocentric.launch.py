from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from yaml import safe_load

parameters_file_path = Path(
    get_package_share_directory('vrpn_client_ros'),
    'config',
    'sample.params.yaml'
)

ego_param_file_path = Path(
    get_package_share_directory('vrpn_client_ros'),
    'config',
    'egocentric.params.yaml'
)

ego_param = None

with open(ego_param_file_path) as f:
    ego_param = safe_load(f)

print(ego_param)

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='vrpn_client_ros',
            executable='vrpn_client_node',
            output='screen',
            emulate_tty=True,
            parameters=[parameters_file_path]),
            
        launch_ros.actions.Node(package="vrpn_client_ros",
                                executable='offset.py',
                                output='screen',
                                emulate_tty=True,
                                parameters=[ego_param]),
        launch_ros.actions.Node(package="vrpn_client_ros",
                                executable='relative_tracking.py',
                                output='screen',
                                emulate_tty=True,
                                parameters=[ego_param]),
    ])
