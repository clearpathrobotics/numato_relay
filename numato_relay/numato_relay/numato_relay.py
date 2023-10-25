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
        super().__init__('NumatoRelay')
        self.service_set_relay = self.create_service(SetRelay, 'set_relay', self.set_relay)
        relay_timer_period = 1.0  # seconds
        gpio_timer_period = 3.0  # seconds
        self.relay_timer = self.create_timer(relay_timer_period, self.relay_timer_callback)
        #self.gpio_timer = self.create_timer(gpio_timer_period, self.gpio_timer_callback)
        self.port = '/dev/ttyACM0'
        self.baud = 19200
        self.serial_port = serial.Serial(self.port, self.baud, timeout=1)
        self.SERIAL_READ_SIZE = 25
        self.serial_lock = Lock()

        self.relay_state_array = []
        self.relay_publisher_0 = None
        self.relay_publisher_1 = None
        self.relay_publisher_2 = None
        self.relay_publisher_3 = None
        self.relay_publisher_4 = None
        self.relay_publisher_5 = None
        self.relay_publisher_6 = None
        self.relay_publisher_7 = None

        self.gpio_state_array = []
        self.gpio_publisher_0 = None
        self.gpio_publisher_1 = None
        self.gpio_publisher_2 = None
        self.gpio_publisher_3 = None
        self.gpio_publisher_4 = None
        self.gpio_publisher_5 = None
        self.gpio_publisher_6 = None
        self.gpio_publisher_7 = None
        

    def set_relay(self, request, response):
        response.relay_response_string = ' '
        response.relay_response_bool = False
        if (type(request.relay_state) is bool):     
            if (type(request.relay_channel) is int):
                if (not (request.relay_channel < 0) ):
                    if ( not(request.relay_channel > (self.get_number_of_relays()-1) ) ):
                        if (not (self.get_relay_state(request.relay_channel) == request.relay_state) ): # Check if the requested relay_channel is already set to the requested relay_state
                            if (request.relay_state):
                                relay_state = 'on' # 'on' or 'off' of whether the relay should be Closed or Open ( on = True = Closed = relay powered )
                                response.relay_response_bool = True
                                response.relay_response_string = 'Relay has been turned on.'
                            else:
                                relay_state = 'off'
                                response.relay_response_bool = False
                                response.relay_response_string = 'Relay has been turned off.'
                            self.change_relay_state(request.relay_channel, request.relay_state)
                            self.serial_lock.acquire()
                            self.serial_port.write(f"relay {relay_state} {request.relay_channel}\n\r".encode("utf-8"))
                            self.serial_port.flush()
                            self.serial_lock.release()
                        else:
                            response.relay_response_string = 'Relay is already set to ' + str(request.relay_state)
                    else:
                        response.relay_response_string = 'The requested relay_channel is larger than what this Relay PCBA supports'
                else:
                    response.relay_response_string = 'The requested relay_channel needs to be a positive Integer'
            else:
                response.relay_response_string = 'The requested relay_channel needs to be an Integer'
        else:
            response.relay_response_string = 'The requested relay_state needs to be an Bool'
        return response


    def read_relay(self):
        index_count = 0
        while True:
            self.serial_lock.acquire()
            self.serial_port.write(f"relay read {index_count} \n\r".encode("utf-8"))
            response = str(self.serial_port.read(self.SERIAL_READ_SIZE))
            self.serial_port.flush()
            self.serial_lock.release()
            if 'on' in response:
                self.set_relay_state(index_count, True)
            elif 'off' in response:
                self.set_relay_state(index_count, False)
            else:
                break
            index_count += 1


    def change_relay_state(self, index, content):
        self.relay_state_array[index] = content


    def set_relay_state(self, index, content):
        self.relay_state_array.insert(index, content)


    def get_relay_state(self, index):
        return self.relay_state_array[index]


    def get_number_of_relays(self):
        return len(self.relay_state_array)
    

    def update_relay_publishers(self):
        number_of_relays = self.get_number_of_relays()
        if ( number_of_relays > 0 ):
            self.relay_publisher_0 = self.create_publisher(Bool, 'numato_relay_state_0', 10)
        if ( number_of_relays > 1 ):
            self.relay_publisher_1 = self.create_publisher(Bool, 'numato_relay_state_1', 10)
        if ( number_of_relays > 2 ):
            self.relay_publisher_2 = self.create_publisher(Bool, 'numato_relay_state_2', 10)
        if ( number_of_relays > 3 ):
            self.relay_publisher_3 = self.create_publisher(Bool, 'numato_relay_state_3', 10)
        if ( number_of_relays > 4 ):
            self.relay_publisher_4 = self.create_publisher(Bool, 'numato_relay_state_4', 10)
        if ( number_of_relays > 5 ):
            self.relay_publisher_5 = self.create_publisher(Bool, 'numato_relay_state_5', 10)
        if ( number_of_relays > 6 ):
            self.relay_publisher_6 = self.create_publisher(Bool, 'numato_relay_state_6', 10)
        if ( number_of_relays > 7 ):
            self.relay_publisher_7 = self.create_publisher(Bool, 'numato_relay_state_7', 10)


    def read_gpio(self):
        index_count = 0
        while True:
            self.serial_lock.acquire()
            self.serial_port.write(f"gpio read {index_count} \n\r".encode("utf-8"))
            response = str(self.serial_port.read(self.SERIAL_READ_SIZE))
            self.serial_port.flush()
            self.serial_lock.release()
            response = response.lstrip(response[0:13])
            if '1' in response:
                self.set_gpio_state(index_count, True)
            elif '0' in response:
                self.set_gpio_state(index_count, False)
            else:
                break
            index_count += 1


    def update_gpio(self):
        for index_count in range(self.get_number_of_gpio()):
            self.serial_lock.acquire()
            self.serial_port.write(f"gpio read {index_count} \n\r".encode("utf-8"))
            response = str(self.serial_port.read(self.SERIAL_READ_SIZE))
            self.serial_port.flush()
            self.serial_lock.release()
            response = response.lstrip(response[0:13])
            if '1' in response:
                self.set_gpio_state(index_count, True)
            elif '0' in response:
                self.set_gpio_state(index_count, False)
            else:
                break


    def change_gpio_state(self, index, content):
        self.gpio_state_array[index] = content


    def set_gpio_state(self, index, content):
        self.gpio_state_array.insert(index, content)


    def get_gpio_state(self, index):
        return self.gpio_state_array[index]


    def get_number_of_gpio(self):
        return len(self.gpio_state_array)


    def update_gpio_publishers(self):
        number_of_gpio = self.get_number_of_gpio()
        if ( number_of_gpio > 0 ):
            self.gpio_publisher_0 = self.create_publisher(Bool, 'numato_gpio_state_0', 10)
        if ( number_of_gpio > 1 ):
            self.gpio_publisher_1 = self.create_publisher(Bool, 'numato_gpio_state_1', 10)
        if ( number_of_gpio > 2 ):
            self.gpio_publisher_2 = self.create_publisher(Bool, 'numato_gpio_state_2', 10)
        if ( number_of_gpio > 3 ):
            self.gpio_publisher_3 = self.create_publisher(Bool, 'numato_gpio_state_3', 10)
        if ( number_of_gpio > 4 ):
            self.gpio_publisher_4 = self.create_publisher(Bool, 'numato_gpio_state_4', 10)
        if ( number_of_gpio > 5 ):
            self.gpio_publisher_5 = self.create_publisher(Bool, 'numato_gpio_state_5', 10)
        if ( number_of_gpio > 6 ):
            self.gpio_publisher_6 = self.create_publisher(Bool, 'numato_gpio_state_6', 10)
        if ( number_of_gpio > 7 ):
            self.gpio_publisher_7 = self.create_publisher(Bool, 'numato_gpio_state_7', 10)


    def relay_timer_callback(self):
        if ( not(self.relay_publisher_0 == None) ):
            msg_0 = Bool()
            msg_0.data = self.get_relay_state(0)
            self.relay_publisher_0.publish(msg_0)
            self.get_logger().info('Publishing relay 0: "%s"' % msg_0.data)

        if ( not(self.relay_publisher_1 == None) ):
            msg_1 = Bool()
            msg_1.data = self.get_relay_state(1)
            self.relay_publisher_1.publish(msg_1)
            self.get_logger().info('Publishing relay 1: "%s"' % msg_1.data)

        if ( not(self.relay_publisher_2 == None) ):
            msg_2 = Bool()
            msg_2.data = self.get_relay_state(2)
            self.relay_publisher_2.publish(msg_2)
            self.get_logger().info('Publishing relay 2: "%s"' % msg_2.data)

        if ( not(self.relay_publisher_3 == None) ):
            msg_3 = Bool()
            msg_3.data = self.get_relay_state(3)
            self.relay_publisher_3.publish(msg_3)
            self.get_logger().info('Publishing relay 3: "%s"' % msg_3.data)

        if ( not(self.relay_publisher_4 == None) ):
            msg_4 = Bool()
            msg_4.data = self.get_relay_state(4)
            self.relay_publisher_4.publish(msg_4)
            self.get_logger().info('Publishing relay 4: "%s"' % msg_4.data)

        if ( not(self.relay_publisher_5 == None) ):
            msg_5 = Bool()
            msg_5.data = self.get_relay_state(5)
            self.relay_publisher_5.publish(msg_5)
            self.get_logger().info('Publishing relay 5: "%s"' % msg_5.data)

        if ( not(self.relay_publisher_6 == None) ):
            msg_6 = Bool()
            msg_6.data = self.get_relay_state(6)
            self.relay_publisher_6.publish(msg_6)
            self.get_logger().info('Publishing relay 6: "%s"' % msg_6.data)

        if ( not(self.relay_publisher_7 == None) ):
            msg_7 = Bool()
            msg_7.data = self.get_relay_state(7)
            self.relay_publisher_7.publish(msg_7)
            self.get_logger().info('Publishing relay 7: "%s"' % msg_7.data)


    def gpio_timer_callback(self):
        self.update_gpio()

        if ( not(self.gpio_publisher_0 == None) ):
            gpio_msg_0 = Bool()
            gpio_msg_0.data = self.get_gpio_state(0)
            self.gpio_publisher_0.publish(gpio_msg_0)
            self.get_logger().info('Publishing GPIO 0: "%s"' % gpio_msg_0.data)

        if ( not(self.gpio_publisher_1 == None) ):
            gpio_msg_1 = Bool()
            gpio_msg_1.data = self.get_gpio_state(1)
            self.gpio_publisher_1.publish(gpio_msg_1)
            self.get_logger().info('Publishing GPIO 1: "%s"' % gpio_msg_1.data)

        if ( not(self.gpio_publisher_2 == None) ):
            gpio_msg_2 = Bool()
            gpio_msg_2.data = self.get_gpio_state(2)
            self.gpio_publisher_2.publish(gpio_msg_2)
            self.get_logger().info('Publishing GPIO 2: "%s"' % gpio_msg_2.data)

        if ( not(self.gpio_publisher_3 == None) ):
            gpio_msg_3 = Bool()
            gpio_msg_3.data = self.get_gpio_state(3)
            self.gpio_publisher_3.publish(gpio_msg_3)
            self.get_logger().info('Publishing GPIO 3: "%s"' % gpio_msg_3.data)

        if ( not(self.gpio_publisher_4 == None) ):
            gpio_msg_4 = Bool()
            gpio_msg_4.data = self.get_gpio_state(4)
            self.gpio_publisher_4.publish(gpio_msg_4)
            self.get_logger().info('Publishing GPIO 4: "%s"' % gpio_msg_4.data)

        if ( not(self.gpio_publisher_5 == None) ):
            gpio_msg_5 = Bool()
            gpio_msg_5.data = self.get_gpio_state(5)
            self.gpio_publisher_5.publish(gpio_msg_5)
            self.get_logger().info('Publishing GPIO 5: "%s"' % gpio_msg_5.data)

        if ( not(self.gpio_publisher_6 == None) ):
            gpio_msg_6 = Bool()
            gpio_msg_6.data = self.get_gpio_state(6)
            self.gpio_publisher_6.publish(gpio_msg_6)
            self.get_logger().info('Publishing GPIO 6: "%s"' % gpio_msg_6.data)

        if ( not(self.gpio_publisher_7 == None) ):
            gpio_msg_7 = Bool()
            gpio_msg_7.data = self.get_gpio_state(7)
            self.gpio_publisher_7.publish(gpio_msg_7)
            self.get_logger().info('Publishing GPIO 7: "%s"' % gpio_msg_7.data)


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


if __name__ == '__main__':
    main()
