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
import sys
import time
import copy

class MapProcessor(Node):
    def __init__(self):
        super().__init__('map_processor')

        # Number of slices (vertical)
        self.num_slices = 2

        # File to store the processed slices
        self.output_file = '/ws/map_data/splitted_map.pkl'
        
        # Create subscriber (use any QoS you need)
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/localization/map',
            self.map_callback,
            1
        )
        
        self.got_map = False
        self.get_logger().info("MapProcessor node started; waiting for /localization/map...")

    def map_callback(self, msg: OccupancyGrid):
        if self.got_map:
            return  # Process only the first map message, then ignore the rest

        self.got_map = True
        self.get_logger().info("Received OccupancyGrid. Beginning processing...")

        # Log input size
        in_width, in_height = msg.info.width, msg.info.height
        in_len = len(msg.data)
        self.get_logger().info(f"Incoming map width={in_width}, height={in_height}, data_length={in_len}")

        start_time = time.time()

        # 1) Lower the map
        msg.info.origin.position.z -= 1.0

        # 2) Split into slices
        slices = self._split_grid(msg, self.num_slices)

        elapsed = time.time() - start_time
        self.get_logger().info(f"Lower + Split took {elapsed:.3f} seconds.")

        # 3) Save to disk
        with open(self.output_file, 'wb') as f:
            pickle.dump(slices, f)
        self.get_logger().info(f"Saved {len(slices)} slices to '{self.output_file}'")

        # 4) Shut down (we only wanted to process once)
        self.get_logger().info("Processing complete. Shutting down node.")
        rclpy.shutdown()

    def _split_grid(self, msg: OccupancyGrid, num_slices: int):
        """Split an OccupancyGrid into num_slices vertical slices."""
        old_width = msg.info.width
        old_height = msg.info.height
        resolution = msg.info.resolution
        old_data = msg.data

        if num_slices < 1:
            self.get_logger().error("num_slices must be >= 1. Returning original map.")
            return [msg]

        if old_width % num_slices != 0:
            self.get_logger().warn(
                f"Map width={old_width} not evenly divisible by num_slices={num_slices}. The last slice may be smaller."
            )

        slice_width = old_width // num_slices
        slices = []

        for i in range(num_slices):
            start_col = i * slice_width
            end_col = (i + 1) * slice_width if i < num_slices - 1 else old_width
            actual_slice_width = end_col - start_col

            new_data = [0] * (actual_slice_width * old_height)
            for row in range(old_height):
                for col in range(start_col, end_col):
                    old_idx = row * old_width + col
                    new_idx = row * actual_slice_width + (col - start_col)
                    new_data[new_idx] = old_data[old_idx]

            sub_msg = OccupancyGrid()
            sub_msg.header = copy.deepcopy(msg.header)
            sub_msg.info.resolution = resolution
            sub_msg.info.width = actual_slice_width
            sub_msg.info.height = old_height
            sub_msg.info.origin = copy.deepcopy(msg.info.origin)
            # Shift the origin in X by how many columns we've moved
            sub_msg.info.origin.position.x += (start_col * resolution)
            sub_msg.data = new_data

            slices.append(sub_msg)

            self.get_logger().info(
                f"Slice {i+1}/{num_slices}: width={sub_msg.info.width}, height={sub_msg.info.height}, "
                f"data_length={len(sub_msg.data)}"
            )

        return slices

def main(args=None):
    rclpy.init(args=args)
    node = MapProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
