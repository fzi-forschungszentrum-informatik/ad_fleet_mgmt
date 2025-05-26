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
import pickle
import os

class StaticTfSaver(Node):
    def __init__(self):
        super().__init__('static_tf_saver')

        self.target_parent = 'map'
        self.target_child = 'cart'
        self.output_file = '/ws/map_data/map_to_cart.pkl'

        self.sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_callback,
            10  # QoS not critical here
        )
        self.get_logger().info("Waiting for static transform map → cart...")

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id == self.target_parent and
                transform.child_frame_id == self.target_child):

                with open(self.output_file, 'wb') as f:
                    pickle.dump(transform, f)

                self.get_logger().info(f"Saved static TF {self.target_parent} → {self.target_child} to {self.output_file}")
                rclpy.shutdown()
                break

def main(args=None):
    rclpy.init(args=args)
    node = StaticTfSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
