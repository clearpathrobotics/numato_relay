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
from numato_relay_interfaces.srv import GetRelay, SetRelay
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
        self.service_get_relay = self.create_service(GetRelay, "get_relay", self.get_relay)
        self.relay_timer = self.create_timer(self.relay_timer_period, self.relay_timer_callback)
        # GPIO polling remains disabled.
        # self.gpio_timer = self.create_timer(self.gpio_timer_period, self.gpio_timer_callback)


    def set_relay(self, request, response):
        if not isinstance(request.relay_state, bool):
            message = (
                f"Invalid type: relay_state should be bool, got {type(request.relay_state)}")
            return self.set_relay_error_response(response, message)
        if not isinstance(request.relay_channel, int):
            message = (
                f"Invalid type: relay_channel should be int, got {type(request.relay_channel)}")
            return self.set_relay_error_response(response, message)
        if not self.validate_relay_channel(request.relay_channel, response):
            return response

        channel = request.relay_channel
        if self.relay_state_array[channel] == request.relay_state:
            response.success = True
            response.relay_state = self.relay_state_array[channel]
            response.message = (
                f"Relay {channel} is already {'on' if request.relay_state else 'off'}.")
            return response

        relay_state = "on" if request.relay_state else "off"
        response.success = True
        response.relay_state = request.relay_state
        response.message = f"Relay {channel} has been turned {relay_state}."

        try:
            with self.serial_lock:
                self.serial_port.write(
                    f"relay {relay_state} {channel}\n\r".encode("utf-8"))
                self.serial_port.flush()
                self.relay_state_array[channel] = request.relay_state
        except (serial.SerialException, OSError) as error:
            message = f"Serial communication failed: {error}"
            self.get_logger().error(
                f"set_relay: relay_channel {channel}: {message}")
            return self.set_relay_error_response(response, message)

        return response


    def set_relay_error_response(self, response, message):
        response.success = False
        response.relay_state = False
        response.message = message
        return response


    def validate_relay_channel(self, channel, response):
        if channel in range(len(self.relay_state_array)):
            return True
        message = (
            f"The requested relay_channel {channel} is outside the supported range "
            f"(0..{len(self.relay_state_array) - 1})")
        self.get_logger().warning(f"set_relay: {message}")
        self.set_relay_error_response(response, message)
        return False


    def get_relay(self, request, response):
        if not self.validate_relay_channel(request.relay_channel, response):
            return response

        channel = request.relay_channel
        response.success = True
        response.relay_state = self.relay_state_array[channel]
        response.message = f"Relay {channel} is {'on' if response.relay_state else 'off'}."
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
