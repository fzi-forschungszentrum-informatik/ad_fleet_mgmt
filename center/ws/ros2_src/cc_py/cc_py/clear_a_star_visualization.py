#!/usr/bin/env python3

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
# 
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
# 
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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
# -- END LICENSE BLOCK ------------------------------------------------
#
# ---------------------------------------------------------------------
# !\file
#
# \author  Martin Gontscharow <gontscharow@fzi.de>
# \date    2025-04-03
#
#
# ---------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped

class ClearAStarVisualization(Node):
    def __init__(self):
        super().__init__('clear_a_star_visualization_node')

        vehicles = self.declare_parameter('vehicles', rclpy.Parameter.Type.STRING_ARRAY).value

        self.state_subscribers = {}
        self.marker_publishers = {}
        self.pose_publishers = {}

        self.last_states = {vehicle: None for vehicle in vehicles}

        for vehicle in vehicles:
            state_topic = f'/{vehicle}/execution/debug/switchbox_state_extra'
            self.state_subscribers[vehicle] = self.create_subscription(
                String, state_topic,
                lambda msg, v=vehicle: self.state_callback(msg, v),
                1
            )

            self.marker_publishers[vehicle] = [
                self.create_publisher(MarkerArray, f'/{vehicle}/visualization/planning/free/result', 1),
                self.create_publisher(MarkerArray, f'/{vehicle}/visualization/planning/free/curvature_planning', 1),
                self.create_publisher(MarkerArray, f'/{vehicle}/visualization/planning/free/curvature_optimizing', 1)
            ]

            self.pose_publishers[vehicle] = self.create_publisher(PoseStamped, f'/to_{vehicle}/move_base_free/goal', 1)

    def state_callback(self, msg, vehicle):
        current_state = msg.data
        last_state = self.last_states[vehicle]

        if last_state == "free" and current_state != "free":
            self.clear_markers(vehicle)
            self.reset_pose(vehicle)

        self.last_states[vehicle] = current_state

    def clear_markers(self, vehicle):
        delete_marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker_array.markers.append(delete_marker)

        for pub in self.marker_publishers[vehicle]:
            pub.publish(delete_marker_array)

        self.get_logger().info(f'Cleared MarkerArray topics for vehicle {vehicle}')

    def reset_pose(self, vehicle):
        null_pose = PoseStamped()
        self.pose_publishers[vehicle].publish(null_pose)
        self.get_logger().info(f'Set PoseStamped to null for vehicle {vehicle}')

def main(args=None):
    rclpy.init(args=args)
    node = ClearAStarVisualization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
