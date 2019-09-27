#!/usr/bin/env python
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

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from lex_common_msgs.srv import AudioTextConversation
from audio_common_msgs.msg import AudioData
from voice_interaction_robot_msgs.msg import FulfilledVoiceCommand

"""
 This node does not respond to lex commands until it has been awoken
 the WakeWord class keeps track of whether this node is awake or not. 
"""
class WakeWord():
    last_wake_time = 0

    def __init__(self, time_awake=10, wake_message=None):
        self.time_awake = time_awake
        self.wake_message = wake_message

    def handle_wake_message(self, request):
        wake_message = request.data
        if self.wake_message is None or wake_message == self.wake_message:
            self.awaken()

    def is_awake(self):
        return time.time() < self.last_wake_time + self.time_awake

    def awaken(self):
        self.last_wake_time = time.time()

    def go_to_sleep(self):
        self.last_wake_time = 0


class VoiceInteraction(Node):
    def __init__(self, node_name, text_input_topic, audio_input_topic, wake_word_topic,
                 text_output_topic, audio_output_topic, fulfilled_command_topic):
        super().__init__(node_name)
        self.ww = WakeWord()
        self.get_logger().info("Initialized node %s" % node_name)
        self.create_subscription(String, text_input_topic, self.handle_text_input)
        self.create_subscription(AudioData, audio_input_topic, self.handle_audio_input)
        self.create_subscription(String, wake_word_topic, self.handle_wake_message)
        self.text_output_publisher = self.create_publisher(String, "/" + node_name + text_output_topic, queue_size=5)
        self.audio_output_publisher = self.create_publisher(AudioData, "/" + node_name + audio_output_topic, queue_size=5)
        self.fulfilled_command_publisher = self.create_publisher(FulfilledVoiceCommand, 
                                                            "/" + node_name + fulfilled_command_topic,
                                                            queue_size=5)
        self.lex_service = self.create_service(AudioTextConversation, "/lex_node/lex_conversation")
        self.use_polly = self.get_parameter("/voice_interaction/use_polly").value

    def handle_wake_message(self, request):
        self.get_logger().info('Received wake message "%s"' % request.data)
        self.ww.handle_wake_message(request)

    def handle_text_input(self, request):
        if not self.ww.is_awake():
            return None

        user_input = request.data
        self.get_logger().info("Received text input {}".format(user_input))
        empty_audio_request = AudioData(data='')
        lex_response = self.lex_service(content_type='text/plain; charset=utf-8',
                                   accept_type='text/plain; charset=utf-8',
                                   text_request=user_input,
                                   audio_request=empty_audio_request)
        self.handle_lex_response(lex_response)

    def handle_audio_input(self, request):
        if not self.ww.is_awake():
            return None

        audio_data = request.data
        accept_type = 'text/plain; charset=utf-8'
        if self.use_polly:
            accept_type = 'audio/pcm'
        lex_response = self.lex_service(content_type='audio/x-l16; sample-rate=16000; channel-count=1',
                                   accept_type=accept_type,
                                   text_request=None,
                                   audio_request=AudioData(data=audio_data))
        self.handle_lex_response(lex_response)

    def handle_lex_response(self, lex_response):
        self.ww.awaken()  # Stay awake after each lex response for further commands
        if lex_response.dialog_state not in ("Fulfilled", "ReadyForFulfillment"):
            self.publish_lex_response(lex_response)
            return

        self.get_logger().info("Performing intent: %s" % lex_response.intent_name)
        fulfilled_command = FulfilledVoiceCommand()
        fulfilled_command.intent_name = lex_response.intent_name
        fulfilled_command.slots = lex_response.slots
        self.fulfilled_command_publisher.publish(fulfilled_command)
        self.publish_lex_response(lex_response)

    def publish_lex_response(self, lex_response):
        if len(lex_response.text_response) > 0:
            self.text_output_publisher.publish(lex_response.text_response)
        if len(lex_response.audio_response.data) > 0:
            self.audio_output_publisher.publish(lex_response.audio_response)


def main():
    rclpy.init()
    voice_interaction = VoiceInteraction(node_name="voice_interaction_node",
                                            text_input_topic="/text_input",
                                            audio_input_topic="/audio_input",
                                            wake_word_topic="/wake_word",
                                            text_output_topic="/text_output",
                                            audio_output_topic="/audio_output",
                                            fulfilled_command_topic="/fulfilled_command")
    rclpy.spin(voice_interaction)


if __name__ == '__main__':
    main()