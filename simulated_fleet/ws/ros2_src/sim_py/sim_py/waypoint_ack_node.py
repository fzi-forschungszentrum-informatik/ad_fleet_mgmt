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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from datetime import datetime
from rcl_interfaces.msg import ParameterDescriptor


class WaypointAckNode(Node):
    def __init__(self):
        super().__init__('waypoint_ack_node')

        self.declare_parameter(
            'vehicle_name',
            '',
            ParameterDescriptor(
                description='Name of the vehicle',
                dynamic_typing=False
            )
        )
        self.vehicle_name = self.get_parameter('vehicle_name').get_parameter_value().string_value

        self.ack_pub = self.create_publisher(String, '/waypoint_ack', 10)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/move_base_free/goal',
            self.goal_callback,
            10
        )

        self.get_logger().info("WaypointAckNode initialized and listening on /move_base_free/goal")

    def goal_callback(self, msg: PoseStamped):
        current_time = datetime.now().strftime("%H:%M:%S")
        ack_msg = f"I ({self.vehicle_name}) received a Waypoint at time {current_time}."
        self.ack_pub.publish(String(data=ack_msg))
        self.get_logger().info(ack_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointAckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("WaypointAckNode shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
