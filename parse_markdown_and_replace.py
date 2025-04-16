#!/usr/bin/env python3
# parse_markdown_and_replace.py

"""
parse_markdown_and_replace.py

A simple script that parses a markdown file with code blocks and replaces files.
"""

import os
import sys
import re
import argparse
from datetime import datetime

def log_info(msg, silent=False):
    if not silent:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"{timestamp} [INFO] {msg}")

def log_error(msg):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"{timestamp} [ERROR] {msg}", file=sys.stderr)

def should_process_file(filepath, include_type):
    """Check if the file should be processed based on include type"""
    filename = os.path.basename(filepath)

    if include_type == "ALL":
        return True
    elif include_type == "C":
        return filename.endswith(".c")
    elif include_type == "H":
        return filename.endswith(".h")
    elif include_type == "CMAKE":
        return filename == "CMakeLists.txt"
    elif include_type == "KCONFIG":
        return filename.startswith("Kconfig")
    else:
        return False

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Parse markdown file and replace files')
    parser.add_argument('markdown_file', help='Path to markdown file')
    parser.add_argument('--silent', action='store_true', help='Minimize output')
    parser.add_argument('--include-type', default='ALL',
                        choices=['C', 'H', 'CMAKE', 'KCONFIG', 'ALL'],
                        help='Filter by file type')
    parser.add_argument('--debug', action='store_true', help='Show debug information')

    args = parser.parse_args()

    # Normalize include type
    include_type = args.include_type.upper()

    log_info(f"Processing markdown file: {args.markdown_file}", args.silent)
    log_info(f"Include filter: {include_type}", args.silent)

    try:
        # Read the markdown file
        with open(args.markdown_file, 'r') as f:
            content = f.read()
    except Exception as e:
        log_error(f"Failed to read file {args.markdown_file}: {str(e)}")
        return 1

    # Use regex to find file blocks
    pattern = r'# (.+?)\n
