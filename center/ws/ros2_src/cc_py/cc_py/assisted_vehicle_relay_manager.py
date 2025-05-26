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
from std_msgs.msg import Empty, String

class AssistedVehicleRelayManager(Node):
    def __init__(self):
        super().__init__('assisted_vehicle_relay_manager')

        self.vehicles = self.declare_parameter('vehicles', rclpy.Parameter.Type.STRING_ARRAY).value

        self.assisted_vehicle = None
        self.vehicle_publishers = {}
        self.reset_publishers = {}

        self.init_publishers()

        self.create_subscription(String, '/assisted_vehicle', self.vehicle_callback, 1)
        self.create_subscription(PoseStamped, '/move_base_free/goal', self.pose_callback, 1)
        self.create_subscription(Empty, '/planning/free/reset', self.reset_callback, 1)

        self.get_logger().info("Assisted Vehicle Relay Manager Node Started.")

    def init_publishers(self):
        for vehicle in self.vehicles:
            self.vehicle_publishers[vehicle] = self.create_publisher(
                PoseStamped, f'/to_{vehicle}/move_base_free/goal', 1
            )
            self.reset_publishers[vehicle] = self.create_publisher(
                Empty, f'/to_{vehicle}/planning/free/reset', 1
            )

    def vehicle_callback(self, msg):
        self.assisted_vehicle = msg.data
        self.get_logger().info(f"Selected vehicle: {self.assisted_vehicle}")

    def pose_callback(self, pose_stamped):
        if self.assisted_vehicle in self.vehicle_publishers:
            self.get_logger().info(f"Publishing goal to {self.assisted_vehicle}")
            self.vehicle_publishers[self.assisted_vehicle].publish(pose_stamped)
        else:
            self.get_logger().warn(f"Cannot send goal. No such vehicle: {self.assisted_vehicle}")

    def reset_callback(self, msg):
        empty = Empty()
        if self.assisted_vehicle in self.reset_publishers:
            self.get_logger().info(f"Resetting goal for {self.assisted_vehicle}")
            self.reset_publishers[self.assisted_vehicle].publish(empty)
        else:
            self.get_logger().warn(f"Cannot reset goal. No such vehicle: {self.assisted_vehicle}")

def main(args=None):
    rclpy.init(args=args)
    manager = AssistedVehicleRelayManager()
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()