#!/usr/bin/env python3
# Software License Agreement (BSD)
#
# @author    Rhys Faultless <rfaultless@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import serial
from threading import Lock
from numato_relay_interfaces.srv import SetRelay
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class NumatoRelay(Node):
    def __init__(self):
        super().__init__("NumatoRelay")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 19200)
        self.declare_parameter("relay_timer_period", 1.0)  # seconds
        self.declare_parameter("gpio_timer_period", 3.0)  # seconds

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.relay_timer_period = self.get_parameter(
            "relay_timer_period").get_parameter_value().double_value
        self.gpio_timer_period = self.get_parameter(
            "gpio_timer_period").get_parameter_value().double_value

        self.serial_port = serial.Serial(self.port, self.baud, timeout=1)
        self.SERIAL_READ_SIZE = 25
        self.serial_lock = Lock()

        self.relay_state_array = []
        self.relay_publishers = []
        self.gpio_state_array = []
        self.gpio_publishers = []

        self.service_set_relay = self.create_service(SetRelay, "set_relay", self.set_relay)
        self.relay_timer = self.create_timer(self.relay_timer_period, self.relay_timer_callback)
        # GPIO polling remains disabled.
        # self.gpio_timer = self.create_timer(self.gpio_timer_period, self.gpio_timer_callback)


    def set_relay(self, request, response):
        response.relay_response_bool = False
        num_relays = len(self.relay_state_array)

        if not isinstance(request.relay_state, bool):
            response.relay_response_string = f"Invalid type: relay_state should be bool, got {type(request.relay_state)}"
            return response
        if not isinstance(request.relay_channel, int):
            response.relay_response_string = f"Invalid type: relay_channel should be int, got {type(request.relay_channel)}"
            return response
        if request.relay_channel not in range(num_relays):
            response.relay_response_string = f"The requested relay_channel {request.relay_channel} not in supported range (0..{num_relays - 1})"
            return response

        relay_state = "on" if request.relay_state else "off"
        response.relay_response_bool = request.relay_state
        response.relay_response_string = f"Relay {request.relay_channel} has been turned {relay_state}."

        with self.serial_lock:
            self.serial_port.write(f"relay {relay_state} {request.relay_channel}\n\r".encode("utf-8"))
            self.serial_port.flush()
            self.relay_state_array[request.relay_channel] = request.relay_state

        return response


    def serial_query(self, command):
        """Write a command and read its response under the serial lock."""
        with self.serial_lock:
            self.serial_port.write(command.encode("utf-8"))
            response = str(self.serial_port.read(self.SERIAL_READ_SIZE))
            self.serial_port.flush()
        return response


    def read_relay(self):
        index_count = 0
        while True:
            response = self.serial_query(f"relay read {index_count} \n\r")
            if "on" in response:
                self.relay_state_array.insert(index_count, True)
            elif "off" in response:
                self.relay_state_array.insert(index_count, False)
            else:
                break
            index_count += 1


    def update_relay_publishers(self):
        number_of_relays = len(self.relay_state_array)
        for i in range(number_of_relays):
            self.relay_publishers.append(self.create_publisher(Bool, f'numato_relay_state_{i}', 10))


    def read_gpio_channel(self, channel):
        """Return the GPIO state, or None when no value is recognized."""
        response = self.serial_query(f"gpio read {channel} \n\r")
        response = response.lstrip(response[0:13])
        if "1" in response:
            return True
        if "0" in response:
            return False
        self.get_logger().debug(
            f"No GPIO state recognized for channel {channel}: {response!r}")
        return None


    def read_gpio(self):
        index_count = 0
        while True:
            state = self.read_gpio_channel(index_count)
            if state is None:
                break
            self.gpio_state_array.insert(index_count, state)
            index_count += 1


    def update_gpio(self):
        for index_count in range(len(self.gpio_state_array)):
            state = self.read_gpio_channel(index_count)
            if state is None:
                break
            self.gpio_state_array[index_count] = state


    def update_gpio_publishers(self):
        number_of_gpio = len(self.gpio_state_array)
        for i in range(number_of_gpio):
            self.gpio_publishers.append(self.create_publisher(Bool, f"numato_gpio_state_{i}", 10))


    def relay_timer_callback(self):
        for i, pub in enumerate(self.relay_publishers):
            msg = Bool()
            msg.data = self.relay_state_array[i]
            pub.publish(msg)
            self.get_logger().info(f"Publishing relay {i}: '{msg.data}'")


    def gpio_timer_callback(self):
        self.update_gpio()

        for i, pub in enumerate(self.gpio_publishers):
            gpio_msg = Bool()
            gpio_msg.data = self.gpio_state_array[i]
            pub.publish(gpio_msg)
            self.get_logger().info(f"Publishing GPIO {i}: '{gpio_msg.data}'")


def main():
    rclpy.init()
    numato_relay = NumatoRelay()
    numato_relay.read_relay()
    numato_relay.update_relay_publishers()
    numato_relay.read_gpio()
    numato_relay.update_gpio_publishers()
    rclpy.spin(numato_relay)
    numato_relay.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
