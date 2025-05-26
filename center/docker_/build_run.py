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

import os
import sys
import subprocess
import shlex
import argparse

project_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(project_dir)

from utils.getters import *
from docker_.build import main as build

def run(additional_run_arguments='-it', run_command='bash', build_ros2_ws=True, check_ros2_ws_dependencies=True):
    # Use shlex.split to safely parse additional_run_arguments and run_command
    additional_run_arguments_parts = shlex.split(additional_run_arguments)

    if not build_ros2_ws:
        additional_run_arguments_parts.append('-e')
        additional_run_arguments_parts.append('BUILD_ROS2_WS=false')
    if not check_ros2_ws_dependencies:
        additional_run_arguments_parts.append('-e')
        additional_run_arguments_parts.append('check_ws_dependencies=false')

    # If exec_command is a single command (not already wrapped for bash), wrap it
    if isinstance(run_command, str):
        run_command_parts = shlex.split(run_command)
    else:
        run_command_parts = run_command  # Assume it's already a list

    docker_command = [
        'docker',
        'run',
        *get_docker_run_args(),
        *additional_run_arguments_parts,
        get_image_name(),
        *run_command_parts
    ]

    print("Executing Docker command:", ' '.join(docker_command))
    subprocess.run(docker_command, check=True)

def main(**run_args):
    build()
    run(**run_args)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a Docker container with specified arguments.")
    parser.add_argument('-a', '--additional_run_arguments', help='Additional arguments to pass to the run command')
    parser.add_argument('-c', '--run_command', help='Command to run in the Docker container')
    parser.add_argument('-b', '--build_ros2_ws', type=bool, help='Flag to build the ROS2 workspace (default: True)')
    parser.add_argument('-d', '--check_ros2_ws_dependencies', type=bool, help='Flag to check ROS2 workspace dependencies (default: True)')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})
