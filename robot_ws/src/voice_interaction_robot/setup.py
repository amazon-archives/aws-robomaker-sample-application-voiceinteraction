import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'voice_interaction_robot'

setup(
    name=package_name,
    version='2.0.0',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', ['config/lex_config.yaml']),
        ('share/' + package_name + '/config', ['config/VoiceInteractionRobot.json']),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['test/integration_test.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='AWS RoboMaker',
    author_email='ros-contributions@amazon.com',
    maintainer='AWS RoboMaker',
    maintainer_email='ros-contributions@amazon.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'AWS RoboMaker robot package that shows how to use a Turtlebot3 with Amazon Lex and Amazon Polly'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_output = voice_interaction_robot.audio_output:main',
            'integration_test = voice_interaction_robot.integration_test:main',
            'voice_command_translator = voice_interaction_robot.voice_command_translator:main',
            'voice_input = voice_interaction_robot.voice_input:main',
            'voice_interaction = voice_interaction_robot.voice_interaction:main',
            'voice_output = voice_interaction_robot.voice_output:main',
        ],
    },
)

