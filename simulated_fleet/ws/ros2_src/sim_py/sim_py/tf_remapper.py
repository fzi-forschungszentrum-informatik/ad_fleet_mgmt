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
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class TFRemapper(Node):
    def __init__(self):
        super().__init__('tf_remapper')
        self.declare_parameter('remap_prefix', '')
        self.remap_prefix = self.get_parameter('remap_prefix').get_parameter_value().string_value

        self.publisher = self.create_publisher(TFMessage, '/tf_remapped', 10)
        self.subscription = self.create_subscription(TFMessage, '/tf', self.callback, 10)

        self.get_logger().info(f"TFRemapper initialized with remap_prefix: {self.remap_prefix}")

    def callback(self, msg: TFMessage):
        remapped_transforms = []

        for transform in msg.transforms:
            new_transform = TransformStamped()
            new_transform.header = transform.header
            new_transform.header.stamp = self.get_clock().now().to_msg()
            new_transform.header.frame_id = f"{self.remap_prefix}_{transform.header.frame_id}"
            new_transform.child_frame_id = f"{self.remap_prefix}_{transform.child_frame_id}"
            new_transform.transform = transform.transform
            remapped_transforms.append(new_transform)

        remapped_msg = TFMessage(transforms=remapped_transforms)
        self.publisher.publish(remapped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TFRemapper shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
