#!/usr/bin/env bash
# find_consecutive_blank_lines.sh
#
# This script searches for all .c and .h files within the main/ and components/
# directories, and reports the first line of each group of 2 or more consecutive
# blank lines. Output is in the format: file:line_number: [second blank line]
# This helps detect and clean up unintended vertical whitespace.

# Ensure awk is available
if ! command -v awk &> /dev/null; then
  echo "Error: awk is not installed. Please install it and try again." >&2
  exit 1
fi

# Collect all relevant .c and .h files from main/ and components/
file_list=$(find main components -type f \( -name '*.c' -o -name '*.h' \))

if [[ -z "$file_list" ]]; then
  echo "No .c or .h files found in main/ or components/ directories." >&2
  exit 0
fi

# Process files with awk to detect 2 or more consecutive blank lines
awk '
  FNR == 1 { blank_count = 0 }
  NF == 0 {
    blank_count++
    if (blank_count == 2)
      print FILENAME ":" FNR ": [second blank line]"
    next
  }
  { blank_count = 0 }
' $file_list

