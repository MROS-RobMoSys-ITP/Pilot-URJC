#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Intelligent Robotics Core S.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Lorena Bajo Rebollo - lorena.bajo@urjc.es

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


class Metacontroller(Node):
    def __init__(self, node_name, mode_name):
        super().__init__('metacontroller')
        cli = self.create_client(ChangeMode, '/'+node_name+'/change_mode')
        while not cli.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        req = ChangeMode.Request()
        req.mode_name = mode_name

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Mode change completed')
            sys.exit()
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

def main(args=None):
    print ("------------------------------")
    print ("Specify the option number:")
    print ("------------------------------")
    print (" 0) Default")
    print (" 1) Battery low")
    print (" 2) Robot lost")
    print (" 3) Obstructed")
    print ("------------------------------")
    print (" 4) Charge completed")
    print (" 5) Robot located")
    print (" 6) Obstructed solved")
    print (" 7) Navigate with PointCloud")
    print ("------------------------------")

    option = input()
    if option == "0":
        print ("Default") 
        mode_name = '__DEFAULT__'
    elif option == "1":
        print ("Battery low.") 
        mode_name = 'LOW_BATTERY'
    elif option == "2":
        print ("Robot lost.") 
        mode_name = 'LOST'
    elif option == "3":
        print ("Obstacle.") 
        mode_name = 'OBSTRUCTED'
    elif option == "4":
        print ("Charge completed.")
        mode_name = 'NORMAL'
    elif option == "5":
        print ("Robot located.")
        mode_name = 'NORMAL'
    elif option == "6":
        print ("Obstacle deleted")
        mode_name = 'NORMAL'
    elif option == "7":
        print ("Navigate with PointCloud")
        mode_name = 'NAVIGATE_W_POINTCLOUD'
    else:
        print("Invalid option.")
        sys.exit()

    rclpy.init(args=args)
    node_name = "pilot"
    node = Metacontroller(node_name, mode_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()