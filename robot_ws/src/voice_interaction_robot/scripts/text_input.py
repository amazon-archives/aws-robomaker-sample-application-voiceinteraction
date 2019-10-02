#!/usr/bin/env python3
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


"""
  This script allows you to send text to your voice interaction robot.
  It publishes to the /text_input topic and listens to the /text_output topic for further instructions.
  
  It automatically keeps the robot awake by publishing a message to /wake_word every n seconds (5 by default)
"""

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TextInput(Node):

    def __init__(self, wake_publish_rate=5):
        super().__init__("text_input_script")
        self.wake_words = ("jarvis", "turtlebot")
        self.text_input_publisher = self.create_publisher(String, "/text_input", 5)
        self.wake_publisher = self.create_publisher(String, "/wake_word", 5)
        self.create_subscription(String, "/text_output", self.display_response)
        self.wake_publish_rate = wake_publish_rate
        wake_thread = threading.Thread(name='wake', target=self.keep_robot_awake)
        wake_thread.daemon = True
        wake_thread.start()

    def keep_robot_awake(self):
        if self.wake_publish_rate == 0:
            return
        while True:
            msg = String(data=self.wake_words[0])
            self.wake_publisher.publish(msg)
            time.sleep(self.wake_publish_rate)

    def send_text(self, text):
        msg = String(data=text)
        if text in self.wake_words:
            self.wake_publisher.publish(msg)
            time.sleep(0.1)
        self.text_input_publisher.publish(msg)

    def display_response(self, data):
        text = data.data
        print(text)

    def get_input(self):
        text = input("")
        if len(text) > 0:
            self.send_text(text)
        self.get_input()


def main():
    usage = """
Usage:
move <forward|backward> <speed>             - Move at meters per second. E.g, "move forward 0.3"
turn <clockwise|counterclockwise> <speed>   - Turn at radians per second. E.g, "turn clockwise 0.78"
stop                                        - Stop all movement

"""
    print(usage)
    rclpy.init()
    text_input = TextInput()
    text_input.get_input()


if __name__ == "__main__":
    main()
