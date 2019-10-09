"""
 Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

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
import time

import numpy as np
import rclpy
from rclpy.node import Node

from voice_interaction_robot_msgs.msg import AudioData
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from ament_index_python.packages import get_package_prefix

WAV_HEADER_LENGTH = 24
AUDIO_ASSETS_DIR = os.path.join(get_package_prefix(
    'voice_interaction_robot'), "assets", "voice_interaction_robot")
AUDIO_EXTENSION = ".wav"


class IntegrationTest():
    """Runs integration tests for text and audio commands

    This runner launches the voice_interaction application and sends text
    commands and audio files to the /text_input and /voice_input topics. 
    It then listens to /cmd_vel to ensure the commands are translated 
    into the correct robot movements. 
    """
    # After each audio command is sent wait this many seconds before sending the next one
    wait_between_audio_commands = 1

    def __init__(self, text_input_publisher, audio_input_publisher, test_type, commands, expected_result):
        self.text_input_publisher = text_input_publisher
        self.audio_input_publisher = audio_input_publisher
        self.test_type = test_type
        self.commands = commands
        self.expected_result = expected_result

    def __str__(self):
        return "IntegrationTest(test_type=%s, commands=%s)" % (self.test_type, self.commands, )

    def run_test(self):
        test_runners = {
            'text': self.run_text_input_test,
            'audio': self.run_audio_input_test
        }
        test_runners[self.test_type]()

    def run_text_input_test(self):
        for command in self.commands:
            self.send_text(command)

    def send_text(self, text):
        self.text_input_publisher.publish(String(data=text))

    def run_audio_input_test(self):
        for filename in self.commands:
            self.send_audio(filename)
            time.sleep(self.wait_between_audio_commands)

    def send_audio(self, filename):
        full_path = os.path.join(AUDIO_ASSETS_DIR, filename + AUDIO_EXTENSION)
        raw_data = self.load_wav_file(full_path)
        if raw_data is not None:
            audio_data = raw_data.tolist()
            self.audio_input_publisher.publish(AudioData(data=audio_data))

    def load_wav_file(self, filepath):
        if not os.path.exists(filepath):
            print("Could not find file " + filepath)
            return None
        data = np.fromfile(open(filepath), np.uint8)[WAV_HEADER_LENGTH:]
        return data

    def check_result(self, logger, result):
        if result == self.expected_result:
            return True
        else:
            logger.info(f"result:{result}")
            logger.info(f"expected result:{self.expected_result}")
            return False


class VoiceInteractionIntegrationTest(Node):
    wake_words = ("jarvis", "turtlebot")
    last_cmd_vel = None
    vinode_start_timeout = 10
    test_sleep_time = 2
    tests = []
    max_retries_per_test = 1
    text_input_topic = "/text_input"
    audio_input_topic = "/audio_input"
    wake_word_topic = "/wake_word"

    def __init__(self):
        super().__init__("integration_test")
        self.text_input_publisher = self.create_publisher(
            String, self.text_input_topic, 5)
        self.audio_input_publisher = self.create_publisher(
            AudioData, self.audio_input_topic, 5)
        self.wake_publisher = self.create_publisher(
            String, self.wake_word_topic, 5)
        self.create_subscription(Twist, "/cmd_vel", self.save_cmd_vel, 5)

    def run_tests(self):
        self.wait_for_voice_interaction_nodes()
        self.wait_for_voice_interaction_services()
        self.wait_for_voice_interaction_node_to_subscribe_to_topic()
        self.load_text_input_tests()
        self.load_audio_input_tests()
        self.get_logger().info(f"Total tests: {len(self.tests)}")
        # Before running any tests, wake the robot and wait for a slightly longer period of time.
        self.wake_robot(2.0)
        for test in self.tests:
            self.run_test(test)

    def run_test(self, test):
        self.wake_robot()
        retry_count = 0
        while retry_count <= self.max_retries_per_test:
            if retry_count > 0:
                self.get_logger().info("Retrying failed test {}".format(test))
                time.sleep(self.test_sleep_time)
            test.run_test()
            rclpy.spin_once(self)
            time.sleep(self.test_sleep_time)
            if test.check_result(self.get_logger(), self.last_cmd_vel):
                self.get_logger().info("test passed")
                return
            else:
                retry_count += 1

        self.get_logger().info("test failed")

    def wait_for_voice_interaction_nodes(self):
        required_nodes = set([
            'lex_node',
            'voice_interaction',
            'voice_command_translator'
        ])
        time.sleep(1)
        while not required_nodes.issubset(self.get_node_names()):
            self.get_logger().info(f"Waiting on nodes to launch...")
            time.sleep(0.1)

    def wait_for_voice_interaction_services(self):
        required_services = set([
            '/lex_conversation'
        ])

        while not required_services.issubset([s[0] for s in self.get_service_names_and_types()]):
            self.get_logger().info(f"Waiting on services to launch...")
            time.sleep(0.1)

    def wait_for_voice_interaction_node_to_subscribe_to_topic(self):
        self.wait_for_subscriber(
            self.text_input_topic, self.vinode_start_timeout)
        self.wait_for_subscriber(
            self.audio_input_topic, self.vinode_start_timeout)
        self.wait_for_subscriber(
            self.wake_word_topic, self.vinode_start_timeout)

    def wait_for_subscriber(self, topic_name, timeout):
        start = time.time()
        while self.count_subscribers(topic_name) < 1:
            if time.time() > start + timeout:
                raise TimeoutError(f"Timed out waiting for subscribers to topic {topic_name}")
            time.sleep(0.1)

    def create_twist(self, linear, angular):
        return Twist(
            linear=Vector3(
                x=float(linear[0]),
                y=float(linear[1]),
                z=float(linear[2])
            ),
            angular=Vector3(
                x=float(angular[0]),
                y=float(angular[1]),
                z=float(angular[2])
            )
        )

    def load_text_input_tests(self):
        text_input_tests = [
            (["move", "forward", "5"], self.create_twist((5, 0, 0), (0, 0, 0))),
            (["move", "backwards", "0.2"],
             self.create_twist((-0.2, 0, 0), (0, 0, 0))),
            (["move forward 2"], self.create_twist((2, 0, 0), (0, 0, 0))),
            (["rotate left 10"], self.create_twist((0, 0, 0), (0, 0, 10))),
            (["rotate", "clockwise",  "5"],
             self.create_twist((0, 0, 0), (0, 0, -5))),
            (["stop"], self.create_twist((0, 0, 0), (0, 0, 0))),
            (["halt"], self.create_twist((0, 0, 0), (0, 0, 0)))
        ]
        for test in text_input_tests:
            (command, expected_result) = test
            integration_test = IntegrationTest(self.text_input_publisher, self.audio_input_publisher,
                                               "text", command, expected_result)
            self.tests.append(integration_test)

    def load_audio_input_tests(self):
        audio_input_tests = [
            (["move-forward-5"], self.create_twist((5, 0, 0), (0, 0, 0))),
            (["turn-clockwise-3"], self.create_twist((0, 0, 0), (0, 0, -3))),
            (["stop"], self.create_twist((0, 0, 0), (0, 0, 0))),
            (["turn", "counterclockwise", "10"],
             self.create_twist((0, 0, 0), (0, 0, 10))),
            (["rotate", "clockwise", "5"], self.create_twist((0, 0, 0), (0, 0, -5))),
        ]
        for test in audio_input_tests:
            (command, expected_result) = test
            integration_test = IntegrationTest(self.text_input_publisher, self.audio_input_publisher,
                                               "audio", command, expected_result)
            self.tests.append(integration_test)

    def wake_robot(self, post_wake_sleep=0.1):
        self.wake_publisher.publish(String(data=self.wake_words[0]))
        time.sleep(post_wake_sleep)

    def save_cmd_vel(self, data):
        self.get_logger().info("Received new cmd_vel: {}".format(data))
        self.last_cmd_vel = data


def main():
    rclpy.init()
    vi_integration_test = VoiceInteractionIntegrationTest()
    vi_integration_test.get_logger().info("Starting integration tests")
    vi_integration_test.run_tests()
    vi_integration_test.get_logger().info("Integration tests complete")


if __name__ == "__main__":
    main()
