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

import actionlib
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from std_msgs.msg import String
from tts.msg import SpeechAction, SpeechGoal


class VoiceOutput(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.create_subscription(String, "/" + node_name + "/speak", self.speak_text)
        self.polly_client = ActionClient(self, SpeechAction, 'tts')


    def speak_text(self, data):
        text = data.data
        timeout = rclpy.Duration(2)
        self.polly_client.wait_for_server(timeout=timeout)
        goal = SpeechGoal()
        goal.text = text
        goal.metadata = '{"voice_id": "Joey"}'
        rclpy.loginfo("Polly Speaking Text: %s" % text)
        self.polly_client.send_goal(goal)


def main():
    rclpy.init()
    voice_output = VoiceOutput(node_name="voice_output")
    rclpy.spin(voice_output)


if __name__ == '__main__':
    main()
