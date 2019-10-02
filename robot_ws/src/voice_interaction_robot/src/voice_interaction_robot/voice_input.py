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
import audioop
import numpy as np
import math
from collections import deque

import rclpy
from rclpy.node import Node
from voice_interaction_robot_msgs.msg import AudioData

class VoiceInput(Node):
    p = pyaudio.PyAudio()
    audio_format = pyaudio.paInt16
    audio_format_width = 2
    chunk_size = 1024
    total_channels = 1
    audio_rate = 16000
    silence_limit_seconds = 1
    previous_audio_seconds = 1
    total_silence_samples = 100
    silence_threshold = 1000

    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Initializing  %s" % node_name)
        self.node_name = node_name
        self.stream = None
        self.input_device_index = None
        self.output_device_index = None
        self.audio_publisher = self.create_publisher(AudioData, "/voice_input_node/audio_input")

    def open_stream(self):
        self.close_stream()
        self.get_logger().info("Opening audio input stream")
        self.stream = self.p.open(format=self.audio_format,
                             channels=self.total_channels,
                             rate=self.audio_rate,
                             input=True,
                             input_device_index=self.input_device_index,
                             frames_per_buffer=self.chunk_size)

    def close_stream(self):
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None

    def listen(self):
        self.open_stream()

        self.get_logger().info("%s listening" % self.node_name)

        current_audio = ''
        chunks_per_second = int(self.audio_rate / self.chunk_size)
        sliding_window = deque(maxlen=self.silence_limit_seconds * chunks_per_second)
        prev_audio = deque(maxlen=self.previous_audio_seconds * chunks_per_second)
        started = False

        while rclpy.ok():
            latest_audio_data = self.stream.read(self.chunk_size)
            sliding_window.append(math.sqrt(abs(audioop.avg(latest_audio_data, self.audio_format_width))))
            if any([x > self.silence_threshold for x in sliding_window]):
                if not started:
                    self.get_logger().info("Sound detected... Recording")
                    started = True
                current_audio += latest_audio_data
            elif started:
                self.get_logger().info("Finished")

                all_audio_data = ''.join(prev_audio) + current_audio
                self.stream.stop_stream()

                audio_bitstream = np.fromstring(all_audio_data, np.uint8)
                audio = audio_bitstream.tolist()
                self.audio_publisher.publish(audio)

                started = False
                sliding_window.clear()
                prev_audio.clear()
                current_audio = ''
                self.stream.start_stream()
                self.get_logger().info("Listening ...")
            else:
                prev_audio.append(latest_audio_data)

    def audio_int(self):
        self.get_record_device_index()
        self.determine_silence_threshold()

    def determine_silence_threshold(self):
        """ Calculates the threshold of when the audio input is loud enough to trigger a recording
            It does this by collecting 50 samples of the background noise and using the
            average of the top 20% as the threshold
        """
        loudest_sound_cohort_size = 0.2  # Top 20% are counted in the loudest sound group.
        silence_threshold_multiplier = 1.6  # Sounds must be at least 1.6x as loud as the loudest silence

        self.get_logger().info("Getting intensity values from mic.")
        self.open_stream()
        tss = self.total_silence_samples
        values = [math.sqrt(abs(audioop.avg(self.stream.read(self.chunk_size), self.audio_format_width)))
                  for _ in range(tss)]
        values = sorted(values, reverse=True)
        sum_of_loudest_sounds = sum(values[:int(tss * loudest_sound_cohort_size)])
        total_samples_in_cohort = int(tss * loudest_sound_cohort_size)
        average_of_loudest_sounds = sum_of_loudest_sounds / total_samples_in_cohort
        self.get_logger().info("Average audio intensity is %d" % average_of_loudest_sounds)
        self.silence_threshold = average_of_loudest_sounds * silence_threshold_multiplier
        self.get_logger().info("Silence threshold set to %d " % self.silence_threshold)
        self.close_stream()

    def get_record_device_index(self):
        """ Finds the audio device named 'record' configured in ~/.asoundrc.
            If this device is not found, returns None and pyaudio will use
            your machines default device
        """

        self.get_logger().info("Attempting to find device named 'record'")
        for i in range(self.p.get_device_count()):
            device = self.p.get_device_info_by_index(i)
            if device['name'] == 'record':
                self.get_logger().info("Found device 'record' at index %d" % i)
                self.input_device_index = i
                return
        self.get_logger().info("Could not find device named 'record', falling back to default recording device")


def main():
    rclpy.init()
    voice_input = VoiceInput(node_name="voice_input")
    voice_input.audio_int()
    voice_input.listen()


if __name__ == '__main__':
    main()
