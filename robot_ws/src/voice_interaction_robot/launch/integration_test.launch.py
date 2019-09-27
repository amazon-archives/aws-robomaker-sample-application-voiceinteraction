import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='voice_interaction_robot',
            node_executable='integration_test',
            node_name='integration_test',
            output='screen'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'voice_interaction_robot'), 'launch/voice_interaction.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'use_polly': 'false',
                'use_microphone': 'false',
                'use_speaker': 'false',
                'output': 'log'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
