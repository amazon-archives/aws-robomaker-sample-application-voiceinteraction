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

import pyaudio

import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData

class AudioOutput(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.create_subscription(AudioData, "/audio_output", self.play_audio_data)
        self.output_device_index = None
        self.get_output_device_index()

    def play_audio_data(self, request):
        audio_data = request.data
        p = pyaudio.PyAudio()
        audio_format = pyaudio.paInt16
        chunk_size = 512
        audio_rate = 16000
        stream = p.open(format=audio_format,
                        channels=1,
                        rate=audio_rate,
                        input=False,
                        output=True,
                        output_device_index=self.output_device_index,
                        frames_per_buffer=chunk_size)
        stream.write(audio_data)

    def get_output_device_index(self):
        """ Finds the audio device named 'play' configured in ~/.asoundrc.
            If this device is not found, returns None and pyaudio will use
            your machines default device
        """
        p = pyaudio.PyAudio()
        self.get_logger().info("Attempting to find device named 'play'")
        for i in range(p.get_device_count()):
            device = p.get_device_info_by_index(i)
            if device['name'] == 'play':
                self.get_logger().info("Found device 'play' at index %d" % i)
                self.output_device_index = i
                return
        self.get_logger().info("Could not find device named 'play', falling back to default audio output device")


def main():
    rclpy.init()
    audio_output = AudioOutput(node_name="audio_output_node")
    rclpy.spin(audio_output)


if __name__ == '__main__':
    main()
