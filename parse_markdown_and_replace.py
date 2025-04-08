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
    pattern = r'# (.+?)\n```(?:.*?)\n(.*?)```'
    matches = re.findall(pattern, content, re.DOTALL)

    replaced_count = 0
    skipped_count = 0
    error_count = 0

    for file_path, file_content in matches:
        # Clean up the file path
        file_path = file_path.strip()
        if file_path.startswith('./'):
            file_path = file_path[2:]

        # Skip files that don't look like valid paths
        if ' ' in file_path or not file_path:
            if args.debug:
                log_info(f"Skipping invalid path: {file_path}", args.silent)
            skipped_count += 1
            continue

        log_info(f"Found file: {file_path}", args.silent)

        # Check if we should process this file type
        if not should_process_file(file_path, include_type):
            log_info(f"Skipping file (type filter): {file_path}", args.silent)
            skipped_count += 1
            continue

        # Create directory if needed
        dir_path = os.path.dirname(file_path)
        if dir_path and not os.path.exists(dir_path):
            try:
                os.makedirs(dir_path)
                log_info(f"Created directory: {dir_path}", args.silent)
            except Exception as e:
                log_error(f"Failed to create directory {dir_path}: {str(e)}")
                error_count += 1
                continue

        # Write content to file
        try:
            with open(file_path, 'w') as f:
                f.write(file_content)
            log_info(f"Replaced file: {file_path}", args.silent)
            replaced_count += 1
        except Exception as e:
            log_error(f"Failed to write to {file_path}: {str(e)}")
            error_count += 1

    log_info(f"Processing complete. Files replaced: {replaced_count}, skipped: {skipped_count}, errors: {error_count}", args.silent)
    return 0 if error_count == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
