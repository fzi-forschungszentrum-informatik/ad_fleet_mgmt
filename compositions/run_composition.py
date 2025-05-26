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

import yaml
import argparse
import importlib
from concurrent.futures import ThreadPoolExecutor, as_completed
from time import sleep  # Import sleep

def load_config(path):
    with open(path, 'r') as file:
        return yaml.safe_load(file)

def dynamic_import(module_name, function_name='run_catmux'):
    module = importlib.import_module(module_name)
    return getattr(module, function_name)

def run_script(script):
    try:
        script_function = dynamic_import(script['module_path'], script['function'])
        print()
        print(f"Executing {script['module_path']}")
        script_arguments = script.get('arguments', {})  # Get arguments from YAML, default to empty dictionary
        script_function(**script_arguments)  # Pass arguments to the function
    except Exception as e:
        print(f"Error importing or executing {script['module_path']}: {e}")

def run_composition(composition_dir):
    config = load_config(f'{composition_dir}/composition.yaml')
    with ThreadPoolExecutor() as executor:
        futures = []
        for script in config['composition']:
            futures.append(executor.submit(run_script, script))
            sleep(1)  # Delay the start of each script by 1 second
        for future in as_completed(futures):
            future.result()  # Wait for all scripts to complete and handle exceptions if any


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script to run compositions according to a composition.yaml.')
    parser.add_argument('-d', '--composition_dir', required=True, help='Directory which contains the composition.yaml.')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    run_composition(**{k: v for k, v in vars(args).items() if v is not None})