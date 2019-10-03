"""
 Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.

 Permission is hereby granted, free of charge, to any person obtaining a copy of this
 software and associated documentation files (the "Software"), to deal in the Software
 without restriction, including without limitation the rights to use, copy, modify,
 merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import os
import sys

import yaml
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_lex_config = os.path.join(get_package_share_directory('voice_interaction_robot'), 'config', 'lex_config.yaml')

    with open(default_lex_config, 'r') as f:
      config_text = f.read()
    config_yaml = yaml.safe_load(config_text)
    default_aws_region = config_yaml['lex_node']['ros__parameters']['aws_client_configuration']['region']
    default_lex_user_id = config_yaml['lex_node']['ros__parameters']['lex_configuration']['user_id']

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='aws_region',
            default_value=os.environ.get('ROS_AWS_REGION', default_aws_region),
            description='AWS region override, defaults to config .yaml if unset'
        ),
        launch.actions.DeclareLaunchArgument(
            name='output',
            default_value='screen',
            description='ROS stdout output location'
        ),
        launch.actions.DeclareLaunchArgument(
            name='user_id',
            default_value=os.environ.get('LEX_USER_ID', default_lex_user_id),
            description='UserID sent when communicating with the Lex Bot, can be any string. Defauls to value in config .yaml if unset'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='lex_node_name',
            default_value='lex_node'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_polly',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_microphone',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_speaker',
            default_value='false'
        ),
        launch_ros.actions.Node(
            package='voice_interaction_robot',
            node_executable='voice_input',
            node_name='voice_input',
            output='screen',
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                }
            ],
            remappings={
                '/voice_input/audio_input': '/audio_input',
            }.items(),
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_microphone'))
        ),
        launch_ros.actions.Node(
            package='voice_interaction_robot',
            node_executable='voice_output',
            node_name='voice_output',
            output='screen',
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                }
            ],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_speaker'))
        ),
        launch_ros.actions.Node(
            package='voice_interaction_robot',
            node_executable='audio_output',
            node_name='audio_output',
            output='screen',
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                }
            ],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_speaker'))
        ),
        launch_ros.actions.Node(
            package='voice_interaction_robot',
            node_executable='voice_interaction',
            node_name='voice_interaction',
            output='screen',
            additional_env={'PYTHONUNBUFFERED' : '1'},
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                    'use_polly': launch.substitutions.LaunchConfiguration('use_polly')
                },
            ],
            remappings={
                '/voice_interaction/text_output': '/text_output',
                '/voice_interaction/audio_output': '/audio_output'
            }.items()
        ),
        launch_ros.actions.Node(
            package='voice_interaction_robot',
            node_executable='voice_command_translator',
            node_name='voice_command_translator',
            output='screen',
            additional_env={'PYTHONUNBUFFERED' : '1'},
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'lex_node'), 'launch/lex.launch.py')
            ),
            launch_arguments={
                'config_file': get_package_share_directory('voice_interaction_robot') + '/config/lex_config.yaml',
                'node_name': launch.substitutions.LaunchConfiguration('lex_node_name'),
                'output': launch.substitutions.LaunchConfiguration('output')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
