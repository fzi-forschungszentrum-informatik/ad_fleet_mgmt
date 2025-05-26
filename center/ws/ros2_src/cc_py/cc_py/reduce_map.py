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
import sys
import time
import copy
import array

def downsample_slice(node: Node, msg: OccupancyGrid, factor: int, slice_idx: int):
    """
    Downsample an OccupancyGrid by a given integer factor using simple block averaging.
    Converts final data to array('b') for memory efficiency and correct int8 usage.
    """
    old_width = msg.info.width
    old_height = msg.info.height
    old_data = msg.data
    old_data_len = len(old_data)

    size_before_bytes = sys.getsizeof(old_data)
    node.get_logger().info(
        f"Slice #{slice_idx} BEFORE: width={old_width}, height={old_height}, "
        f"data_len={old_data_len}, approx_size={human_readable_size(size_before_bytes)}"
    )

    # Compute the new dimensions
    new_width = old_width // factor
    new_height = old_height // factor
    new_resolution = msg.info.resolution * factor  # so physical coverage stays the same

    # Allocate Python list for averaging
    new_list = [0] * (new_width * new_height)

    for new_row in range(new_height):
        for new_col in range(new_width):
            sum_occupancy = 0
            valid_cells = 0
            # Loop through the factor×factor block in original data
            for i in range(factor):
                for j in range(factor):
                    old_row = new_row * factor + i
                    old_col = new_col * factor + j
                    idx = old_row * old_width + old_col
                    if idx < old_data_len:
                        val = old_data[idx]
                        # If val >= 0, treat as known cell
                        if val >= 0:
                            sum_occupancy += val
                            valid_cells += 1
            # If no known cells, set -1 (unknown)
            avg_val = -1
            if valid_cells > 0:
                avg_val = sum_occupancy // valid_cells
            new_list[new_row * new_width + new_col] = avg_val

    # Convert Python list -> array('b') so it behaves like int8[] and uses less memory
    # OccupancyGrid data is typically in the range [-1, 100], so it's safe in signed char.
    new_data = array.array('b', new_list)

    # Build new OccupancyGrid
    downsampled_msg = OccupancyGrid()
    downsampled_msg.header = copy.deepcopy(msg.header)
    downsampled_msg.info.resolution = new_resolution
    downsampled_msg.info.width = new_width
    downsampled_msg.info.height = new_height
    downsampled_msg.info.origin = copy.deepcopy(msg.info.origin)
    downsampled_msg.data = new_data

    size_after_bytes = sys.getsizeof(new_data)
    node.get_logger().info(
        f"Slice #{slice_idx} AFTER: width={new_width}, height={new_height}, "
        f"data_len={len(new_data)}, approx_size={human_readable_size(size_after_bytes)}"
    )

    return downsampled_msg

def human_readable_size(num_bytes: int) -> str:
    """Return a human‐friendly size string (B, KB, or MB)."""
    if num_bytes < 1024:
        return f"{num_bytes} B"
    elif num_bytes < 1024**2:
        return f"{num_bytes/1024:.2f} KB"
    else:
        return f"{num_bytes/(1024**2):.2f} MB"

def main(args=None):
    rclpy.init(args=args)
    node = Node('occupancy_grid_reducer')

    # Parameters
    node.declare_parameter('input_file', 'splitted_map.pkl')
    node.declare_parameter('output_file', 'splitted_map_reduced.pkl')
    node.declare_parameter('downsample_factor', 2)

    input_file = node.get_parameter('input_file').get_parameter_value().string_value
    output_file = node.get_parameter('output_file').get_parameter_value().string_value
    factor = node.get_parameter('downsample_factor').get_parameter_value().integer_value

    if not os.path.isfile(input_file):
        node.get_logger().error(f"Input file not found: {input_file}")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Load slices
    with open(input_file, 'rb') as f:
        slices = pickle.load(f)
    num_slices = len(slices)
    if num_slices == 0:
        node.get_logger().error("No slices found in the input file. Exiting.")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(
        f"Loaded {num_slices} slice(s) from '{input_file}'. "
        f"Now downsampling each by factor={factor}..."
    )

    # Reduce
    start_time = time.time()
    reduced_slices = []
    for i, slice_msg in enumerate(slices):
        reduced_slice = downsample_slice(node, slice_msg, factor, i+1)
        reduced_slices.append(reduced_slice)
    elapsed = time.time() - start_time

    # Save
    with open(output_file, 'wb') as f:
        pickle.dump(reduced_slices, f)
    node.get_logger().info(
        f"Saved {len(reduced_slices)} downsampled slice(s) to '{output_file}' in {elapsed:.3f} seconds."
    )

    node.get_logger().info("Reducer node finished successfully. Shutting down.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
