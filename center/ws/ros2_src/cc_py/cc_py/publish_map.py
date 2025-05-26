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
from nav_msgs.msg import OccupancyGrid
import pickle
import os

class LatchedMapPublisher(Node):
    def __init__(self):
        super().__init__('latched_map_publisher')

        self.declare_parameter('input_file', 'splitted_map.pkl')
        self.input_file = self.get_parameter('input_file').get_parameter_value().string_value

        if not os.path.isfile(self.input_file):
            self.get_logger().error(f"File not found: {self.input_file}")
            self.get_logger().error("Make sure you ran process_map.py and generated the splitted_map.pkl first!")
            rclpy.shutdown()
            return

        # Load the slices
        with open(self.input_file, 'rb') as f:
            self.slices = pickle.load(f)

        self.num_slices = len(self.slices)
        if self.num_slices == 0:
            self.get_logger().error("No slices found in file. Exiting.")
            rclpy.shutdown()
            return

        # Create latched publishers for each slice
        self.m_publishers = []
        for i in range(self.num_slices):
            topic_name = f'/localization/map_lowered_latched_{i+1}'
            qos_profile = rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,  # latched
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
            )
            pub = self.create_publisher(OccupancyGrid, topic_name, qos_profile)
            self.m_publishers.append(pub)
            self.get_logger().info(f"Created latched publisher for slice {i+1}/{self.num_slices} on topic: {topic_name}")

        # Publish them once
        self._publish_slices()
        self.get_logger().info(f"Published {self.num_slices} latched OccupancyGrid slices. Node will stay alive.")

    def _publish_slices(self):
        for i, slice_msg in enumerate(self.slices):
            self.m_publishers[i].publish(slice_msg)
            self.get_logger().info(f"Published slice {i+1}/{self.num_slices} latched message.")

def main(args=None):
    rclpy.init(args=args)
    node = LatchedMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
