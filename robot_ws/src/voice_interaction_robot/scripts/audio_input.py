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

"""
  This script allows you to load and send audio files to your voice interaction robot.
  It publishes the audio to /audio_input for the robot. It also publishes to to /audio_output so you can hear what is 
  being said on your PC's speakers. 
  
  It automatically keeps the robot awake by publishing a message to /wake_word every n seconds (5 by default)
"""

import rospy
import rospkg
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import numpy as np
import os
import time
import threading

WAV_HEADER_LENGTH = 24
DEFAULT_ASSETS_DIR = rospkg.RosPack().get_path('voice_interaction_robot') + "/assets"
DEFAULT_ASSETS_EXT = ".wav"

audio_input_publisher = rospy.Publisher("/audio_input", AudioData, queue_size=5)
audio_output_publisher = rospy.Publisher("/audio_output", AudioData, queue_size=5)
wake_publisher = rospy.Publisher("/wake_word", String, queue_size=5)


class AudioInput():
    current_directory = DEFAULT_ASSETS_DIR
    default_extension = DEFAULT_ASSETS_EXT
    wake_words = ("jarvis", "turtlebot")

    def __init__(self, wake_publish_rate=5):
        self.wake_publish_rate = wake_publish_rate
        wake_thread = threading.Thread(name='wake', target=self.keep_robot_awake)
        wake_thread.daemon = True
        wake_thread.start()

    def keep_robot_awake(self):
        if self.wake_publish_rate == 0:
            return
        while True:
            wake_publisher.publish(self.wake_words[0])
            time.sleep(self.wake_publish_rate)

    def send_audio(self, data):
        audio_data = data.tolist()
        audio_input_publisher.publish(audio_data)

    def get_input(self):
        command = raw_input("Audio file or command:\n")
        self.process_command(command)
        self.get_input()

    def process_command(self, command):
        if command.startswith("/dir"):
            self.set_current_directory(command)
            return
        if command.startswith("/ext"):
            self.set_default_extension(command)
            return
        if self.command_contains_wake_word(command):
            wake_publisher.publish("wake")
            time.sleep(0.1)
        self.process_filepath(command)

    def set_current_directory(self, command):
        directory = command.split(" ", 1)[1]
        if directory.endswith("/"):
            directory = directory[:-1]
        self.current_directory = directory

    def set_default_extension(self, command):
        extension = command.split(" ", 1)[1]
        if not extension.startswith("."):
            extension = "." + extension
        self.default_extension = extension

    def command_contains_wake_word(self, command):
        return any(wake_word in command for wake_word in self.wake_words)

    def process_filepath(self, filepath):
        full_path = self.current_directory + "/" + filepath + self.default_extension
        audio = self.load_wav_file(full_path)
        if audio is not None:
            self.play_audio_data(audio)
            self.send_audio(audio)

    def load_wav_file(self, filepath):
        if not os.path.exists(filepath):
            print("Could not find file " + filepath)
            return None
        print("Loading file " + filepath)
        data = np.fromfile(open(filepath), np.uint8)[WAV_HEADER_LENGTH:]
        return data

    def play_audio_data(self, data):
        audio_data = data.astype(np.uint8).tostring()
        audio_output_publisher.publish(audio_data)


def main():
    usage = """
Usage:
jarvis / turtlebot          - Wake up the robot
move <direction> <speed>    - Move in a direction at <speed>
turn <direction> <speed>    - Turn clockwise/counterclockwise at <speed>
stop                        - Stop all movement

Helpers
/dir <directory> set the default directory to read audio files from (default: {DEFAULT_ASSETS_DIR})
/ext <extension> set the default extension that all audio files will have (default: {DEFAULT_ASSETS_EXT})
""".format(DEFAULT_ASSETS_DIR=DEFAULT_ASSETS_DIR, DEFAULT_ASSETS_EXT=DEFAULT_ASSETS_EXT)
    print(usage)
    rospy.init_node("audio_input_script", disable_signals=True)
    audio_input = AudioInput()
    audio_input.get_input()


if __name__ == "__main__":
    main()