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

import json
import re
import os

def remove_comments(json_like):
    """Remove C-style comments from a JSON-like string."""
    pattern = r'//.*?$|/\*.*?\*/'
    return re.sub(pattern, '', json_like, flags=re.DOTALL | re.MULTILINE)

def remove_empty_lines(text):
    """Remove empty lines from a text string while keeping line breaks."""
    return "\n".join([line for line in text.splitlines() if line.strip()])

def json_to_dict(path_to_json):
    with open(path_to_json, 'r') as file:
        content = file.read()

        content_no_comments = remove_comments(content)
        # Return parsed JSON
        return json.loads(content_no_comments)

# Function to load template and perform substitutions
def process_template(template_path, substitutions):
    # Read the template content
    with open(template_path, 'r') as file:
        content = file.read()
    # Perform substitutions
    for key, value in substitutions.items():
        content = content.replace(key, value)

    content = remove_comments(content)
    content = remove_empty_lines(content)
    
    return content

def generate_file_from_template(template_path, output_path, substitutions):
    """
    Generate a file from a template by performing variable substitutions.
    
    Args:
        template_path (str): Path to the template file.
        output_path (str): Path to the output file.
        substitutions (dict): Dictionary with keys as placeholders and values as substitutions.
    """
    # Process the template
    processed_content = process_template(template_path, substitutions)

    # Ensure the directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Check if the file already exists
    file_exists = os.path.exists(output_path)

    if file_exists:
        os.remove(output_path)

    # Write the processed content to the new file
    with open(output_path, 'w') as file:
        file.write(processed_content)

    # Inform the user about the action taken
    if file_exists:
        print(f"Existing '{os.path.basename(output_path)}' was replaced in the directory.")
    else:
        print(f"'{os.path.basename(output_path)}' was created in the directory.")

    # Change the file mode to read-only
    os.chmod(output_path, 0o444)