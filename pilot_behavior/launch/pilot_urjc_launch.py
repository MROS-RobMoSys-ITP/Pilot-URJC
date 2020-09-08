# Copyright 2019 Intelligent Robotics Lab
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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pilot_behavior_cmd = Node(
        package='pilot_behavior',
        node_executable='pilot_behavior',
        node_name='pilot_behavior',
        output='screen',
        parameters=[])

    rqt_bica_graph_cmd = Node(
        package='rqt_gui',
        node_executable='rqt_gui',
        node_name='bica_rqt_graph',
        output='screen',
        parameters=[{'-s': 'bica_rqt_graph'}])

    ld = LaunchDescription()

    ld.add_action(pilot_behavior_cmd)
    ld.add_action(rqt_bica_graph_cmd)

    return ld
