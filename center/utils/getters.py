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
# \date    2024-11-13
#
#
# ---------------------------------------------------------------------

import os
import sys
import json

project_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(project_dir)

from utils.files import *

def get_local_config():
    return json_to_dict(f"{project_dir}/config/local.json")

def get_static_docker_run_args():
    local_config = get_local_config()

    template_file_path = f"{project_dir}/config/static_docker_run_args.json.template"

    substitutions = {
        "#uid": str(os.getuid()),
        "#gid": str(os.getgid()),
        "#container_name": local_config['container_name'],
        "#project_dir": project_dir,
        "#network": local_config['network']
    }

    processed_content = process_template(template_file_path, substitutions)
    content_dict = json.loads(processed_content)
    static_docker_run_args = content_dict["static_docker_run_args"]
    return static_docker_run_args

def get_optional_docker_run_args():
    optional_docker_run_args = []

    if get_local_config().get("enable_gui_forwarding"):
        optional_docker_run_args.extend([
            "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
            "-e", "DISPLAY",
        ])

    if get_local_config().get("forward_ssh_agent"):
        ssh_auth_sock = os.getenv('SSH_AUTH_SOCK')
        optional_docker_run_args.extend([
            "-e", "SSH_AUTH_SOCK",
            "-v", f"{ssh_auth_sock}:{ssh_auth_sock}"
        ])
    return optional_docker_run_args

def get_docker_run_args():
    return get_static_docker_run_args() + get_optional_docker_run_args()

def get_image_name():
    return get_local_config()["image_name"]

def get_ros_domain_id():
    return get_local_config()["ros_domain_id"]

def get_container_name():
    return get_local_config()["container_name"]