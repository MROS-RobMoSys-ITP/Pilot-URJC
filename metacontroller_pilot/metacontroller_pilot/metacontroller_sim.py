#!/usr/bin/env python

# Copyright 2020 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: jginesclavero - jonatan.gines@urjc.es

import argparse
import functools
import re
import time
import sys

import rclpy
import rclpy.logging
from rclpy.node import Node

from rqt_gui_py.plugin import Plugin
from std_msgs.msg import Float32, Header
from system_modes.srv import ChangeMode
from rcl_interfaces.msg import Log
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Metacontroller(Node):
    def __init__(self):
        super().__init__('metacontroller')
        self.diagnostics_sub_ = self.create_subscription(
          DiagnosticArray,
          "/metacontroller/diagnostics",
          self.diagnostics_cb, 1)
        self.current_mode = 'NORMAL'
    def change_mode(self, node_name, mode_name):
        cli = self.create_client(ChangeMode, '/'+node_name+'/change_mode')
        while not cli.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        req = ChangeMode.Request()
        req.node_name = node_name
        req.mode_name = mode_name

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Mode change completed')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

    def diagnostics_cb(self, msg):
        for status in msg.status:
            if status.message == "QA status":
                for value in status.values:
                    if value.key == "energy" and float(value.value) < 15.0 and self.current_mode == 'NORMAL':
                        self.current_mode = 'ENERGY_SAVING'
                        self.get_logger().info('Battery low detected, solving contingency...')
                        self.change_mode("pilot", self.current_mode)

def main(args=None):
    rclpy.init(args=args)
    node = Metacontroller()
    node.change_mode("pilot", '__DEFAULT__')
    node.change_mode("pilot", 'NORMAL')
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()