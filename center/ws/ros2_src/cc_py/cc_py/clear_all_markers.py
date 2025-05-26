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
from visualization_msgs.msg import MarkerArray, Marker
from std_srvs.srv import Empty

class MarkerArrayClearService(Node):
    def __init__(self):
        super().__init__('marker_array_clear_service')

        self.srv = self.create_service(Empty, 'clear_all_markers', self.clear_markers_service)
        self.m_publishers = {}

        self.get_logger().info("MarkerArray clear service is ready. Call /clear_all_markers service to clear markers.")

    def create_delete_marker_array(self):
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker_array = MarkerArray()
        delete_marker_array.markers.append(delete_marker)
        return delete_marker_array

    def clear_markers_on_topic(self, topic_name):
        if topic_name not in self.m_publishers:
            qos_profile = rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
            )
            self.m_publishers[topic_name] = self.create_publisher(MarkerArray, topic_name, qos_profile)

        delete_marker_array = self.create_delete_marker_array()
        self.m_publishers[topic_name].publish(delete_marker_array)
        self.get_logger().info(f"Published DELETEALL to {topic_name}")

    def get_marker_array_topics(self):
        topics_and_types = self.get_topic_names_and_types()
        marker_array_topics = [topic for topic, types in topics_and_types if 'visualization_msgs/msg/MarkerArray' in types]
        self.get_logger().info(f"Found {len(marker_array_topics)} MarkerArray topics.")
        return marker_array_topics

    def clear_markers_service(self, request, response):
        marker_array_topics = self.get_marker_array_topics()

        if not marker_array_topics:
            self.get_logger().info("No MarkerArray topics found.")
        else:
            for topic in marker_array_topics:
                self.clear_markers_on_topic(topic)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = MarkerArrayClearService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()