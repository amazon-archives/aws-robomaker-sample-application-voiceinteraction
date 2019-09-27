import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_0'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_1'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_2'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_3'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_4'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_5'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_6'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_7'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_8'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_9'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            node_executable='spawn_model',
            node_name='spawn_soccer_ball_10'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
