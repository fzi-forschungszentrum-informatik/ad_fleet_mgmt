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
# \date    2024-04-03
#
#
# ---------------------------------------------------------------------

import argparse
import os

directory_of_this_script = os.path.dirname(os.path.realpath(__file__))

from build_run import main as build_run


def main(catmux_session_file="demo.yaml", additional_run_arguments="-it", catmux_params="", build_ros2_ws=True, check_ros2_ws_dependencies=True):
    catmux_session_path = f'/ws/run_yamls/{catmux_session_file}'
    inner_command=f"catmux_create_session {catmux_session_path} --session_name center"
    if catmux_params: inner_command += f" --overwrite {catmux_params}"
    run_command=f'/bin/bash -c "{inner_command}"'

    build_run(
       additional_run_arguments=additional_run_arguments,
       run_command=run_command,
       build_ros2_ws=build_ros2_ws,
       check_ros2_ws_dependencies=check_ros2_ws_dependencies)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--catmux_session_file', help='Path to the catmux session file (default: demo.yaml)')
    parser.add_argument('-a', '--additional_run_arguments', help='Additional arguments to pass to the run command')
    parser.add_argument('-p', '--catmux_params', help='Parameters for catmux session creation')
    parser.add_argument('-b', '--build_ros2_ws', type=bool, help='Flag to build the ROS2 workspace (default: True)')
    parser.add_argument('-d', '--check_ros2_ws_dependencies', type=bool, help='Flag to check ROS2 workspace dependencies (default: True)')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})
