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
from visualization_msgs.msg import MarkerArray

class MarkerRemapper(Node):
    def __init__(self):
        super().__init__('marker_remapper')

        self.declare_parameter('frame_prefix', '')
        self.frame_prefix = self.get_parameter('frame_prefix').get_parameter_value().string_value

        self.base_topics = [
            '/visualization/planning/free/curvature_optimizing',
            '/visualization/planning/free/curvature_planning',
            '/visualization/planning/pso/submitted_visualization',
            '/visualization/maneuver/announcements_barriers'
        ]

        self.remapped_publishers = {}

        self.setup_remappers()

    def marker_remapper(self, base_topic):
        topic = base_topic
        remapped_topic = f"{base_topic}_remapped"

        pub = self.create_publisher(MarkerArray, remapped_topic, 1)
        self.remapped_publishers[base_topic] = pub

        def callback(data):
            for marker in data.markers:
                marker.header.frame_id = f"{self.frame_prefix}_{marker.header.frame_id}"
                # marker.header.stamp = self.get_clock().now().to_msg()
            pub.publish(data)

        self.create_subscription(MarkerArray, topic, callback, 1)

    def setup_remappers(self):
        for base_topic in self.base_topics:
            self.marker_remapper(base_topic)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()