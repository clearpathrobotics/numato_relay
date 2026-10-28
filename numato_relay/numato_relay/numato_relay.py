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
import time
from threading import Lock
from numato_relay_interfaces.msg import GpioStates, RelayStates
from numato_relay_interfaces.srv import GetRelay, SetRelay, ToggleRelay
import rclpy
from rclpy.node import Node


class NumatoRelay(Node):
    def __init__(self):
        super().__init__("NumatoRelay")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 19200)
        self.declare_parameter("toggle_duration", 2.0)  # seconds held in opposite state
        self.declare_parameter("relay_timer_period", 1.0)  # seconds
        self.declare_parameter("gpio_timer_period", 3.0)  # seconds

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.toggle_duration = self.get_parameter(
            "toggle_duration").get_parameter_value().double_value
        self.relay_timer_period = self.get_parameter(
            "relay_timer_period").get_parameter_value().double_value
        self.gpio_timer_period = self.get_parameter(
            "gpio_timer_period").get_parameter_value().double_value

        self.serial_port = serial.Serial(self.port, self.baud, timeout=1)
        self.SERIAL_READ_SIZE = 25
        self.serial_lock = Lock()

        self.relay_state_array = []
        self.gpio_state_array = []

        self.relay_publisher = self.create_publisher(RelayStates, "relay_states", 10)
        self.gpio_publisher = self.create_publisher(GpioStates, "gpio_states", 10)
        self.service_set_relay = self.create_service(SetRelay, "set_relay", self.set_relay)
        self.service_get_relay = self.create_service(GetRelay, "get_relay", self.get_relay)
        self.service_toggle_relay = self.create_service(
            ToggleRelay, "toggle_relay", self.toggle_relay)
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
        if not self.validate_relay_channel(request.relay_channel, response, "set_relay"):
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
            self.write_relay(channel, request.relay_state)
        except (serial.SerialException, OSError) as error:
            message = f"Serial communication failed: {error}"
            self.get_logger().error(f"set_relay: relay_channel {channel}: {message}")
            return self.set_relay_error_response(response, message)

        return response


    def set_relay_error_response(self, response, message):
        response.success = False
        response.relay_state = False
        response.message = message
        return response


    def validate_relay_channel(self, channel, response, operation):
        if channel in range(len(self.relay_state_array)):
            return True
        message = (
            f"The requested relay_channel {channel} is outside the supported range "
            f"(0..{len(self.relay_state_array) - 1})")
        self.get_logger().warning(f"{operation}: {message}")
        self.set_relay_error_response(response, message)
        return False


    def toggle_relay(self, request, response):
        if not isinstance(request.relay_state, bool):
            message = (
                f"Invalid type: relay_state should be bool, got {type(request.relay_state)}")
            return self.set_relay_error_response(response, message)
        if not isinstance(request.relay_channel, int):
            message = (
                f"Invalid type: relay_channel should be int, got {type(request.relay_channel)}")
            return self.set_relay_error_response(response, message)
        if not self.validate_relay_channel(request.relay_channel, response, "toggle_relay"):
            return response

        channel = request.relay_channel
        desired = request.relay_state
        try:
            if self.relay_state_array[channel] == desired:
                self.write_relay(channel, not desired)
                # Blocks this node's executor while the relay is power-cycled.
                time.sleep(max(self.toggle_duration, 0.0))
                self.write_relay(channel, desired)
                response.message = (
                    f"Relay {channel} was already {'on' if desired else 'off'}; "
                    f"toggled for {self.toggle_duration:.2f} s and restored.")
            else:
                self.write_relay(channel, desired)
                response.message = (
                    f"Relay {channel} was set to {'on' if desired else 'off'}.")
        except (serial.SerialException, OSError) as error:
            message = f"Serial communication failed: {error}"
            self.get_logger().error(f"toggle_relay: relay_channel {channel}: {message}")
            return self.set_relay_error_response(response, message)

        response.success = True
        response.relay_state = self.relay_state_array[channel]
        return response


    def write_relay(self, channel, state):
        relay_state = "on" if state else "off"
        with self.serial_lock:
            self.serial_port.write(
                f"relay {relay_state} {channel}\n\r".encode("utf-8"))
            self.serial_port.flush()
            self.relay_state_array[channel] = state


    def get_relay(self, request, response):
        if not self.validate_relay_channel(request.relay_channel, response, "get_relay"):
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


    def relay_timer_callback(self):
        msg = RelayStates()
        msg.states = list(self.relay_state_array)
        self.relay_publisher.publish(msg)
        self.get_logger().debug(f"Publishing relay states: {msg.states}")


    def gpio_timer_callback(self):
        self.update_gpio()

        msg = GpioStates()
        msg.states = list(self.gpio_state_array)
        self.gpio_publisher.publish(msg)
        self.get_logger().debug(f"Publishing GPIO states: {msg.states}")


def main():
    rclpy.init()
    numato_relay = NumatoRelay()
    numato_relay.read_relay()
    numato_relay.read_gpio()
    rclpy.spin(numato_relay)
    numato_relay.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
