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
This node processes fulfilled commands from lex and turns them into robot commands
"""

import rclpy
from rclpy.node import Node
from voice_interaction_robot_msgs.msg import FulfilledVoiceCommand
from geometry_msgs.msg import Twist


class VoiceCommandTranslator(Node):
    slots = []

    def __init__(self):
        super().__init__("voice_command_translator")
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(FulfilledVoiceCommand, '/voice_interaction_node/fulfilled_command', self.handle_voice_command, 5)

    def handle_voice_command(self, command):
        self.get_logger().debug("Fulfilled lex command:")
        self.get_logger().debug(command)
        response_handlers = {
            'hello': self.ignore,
            'move': self.move_handler,
            'turn': self.turn_handler,
            'stop': self.stop_handler,
        }
        self.slots = self.convert_slot_array_to_map(command.slots)
        if command.intent_name not in response_handlers:
            self.get_logger().warn("Could not find handler for lex intent " + command.intent_name)
            return
        response_handlers[command.intent_name]()
    
    def convert_slot_array_to_map(self, slot_array):
        return {slot.key: slot.value for slot in slot_array}

    def ignore(self):
        pass

    def stop_handler(self):
        stop_command = Twist()
        self.publish_command(stop_command)
        
    def move_handler(self):
        move_command = self.convert_response_to_move_command()
        self.publish_command(move_command)
        
    def convert_response_to_move_command(self):
        twist = Twist()
        twist.linear.x = float(self.slots['move_rate'])
        if self.slots['move_direction'].startswith("back"):
            twist.linear.x *= -1
        return twist
        
    def turn_handler(self):
        turn_command = self.convert_response_to_turn_command()
        self.publish_command(turn_command)
        
    def convert_response_to_turn_command(self):
        twist = Twist()
        twist.angular.z = float(self.slots['turn_rate'])
        if self.slots['turn_direction'].startswith("clock"):
            twist.angular.z *= -1
        return twist
        
    def publish_command(self, cmd_vel_command):
        self.get_logger().debug("publishing the following to /cmd_vel:")
        self.get_logger().debug(cmd_vel_command)
        self.cmd_vel_publisher.publish(cmd_vel_command)


def main():
    rclpy.init()
    voice_command_translator = VoiceCommandTranslator()
    rclpy.spin(voice_command_translator)


if __name__ == '__main__':
    main()
