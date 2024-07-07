#!/usr/bin/env bash

# Check if .clang-format file exists in the current directory
if [ ! -f .clang-format ]; then
    echo ".clang-format file not found in the base directory."
    exit 1
fi

# Function to format files using clang-format or asmfmt
format_files() {
    local file_extension=$1
    local formatter_command=$2

    find . -type f ! -path "./SDK/*" -name "*.${file_extension}" -print0 | \
        xargs -0 -P8 -I {} sh -c "echo 'Formatting {}'; ${formatter_command} '{}'"
}

# Format .h and .c files using clang-format
format_files "h" "clang-format-17 -i"
format_files "c" "clang-format-17 -i"
format_files "cc" "clang-format-17 -i"

# Format .s files using asmfmt
format_files "s" "asmfmt -w"

echo "Formatting complete."

