import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'turtlebot3_bringup'), 'launch/turtlebot3_robot.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'voice_interaction_robot'), 'launch/voice_interaction.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'use_polly': 'true',
                'use_microphone': 'true',
                'use_speaker': 'true',
                'output': 'log'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
