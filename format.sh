#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status,
# treat unset variables as an error,
# and ensure that the return value of a pipeline is the status of the last command to exit with a non-zero status
set -euo pipefail

# Uncomment the following line for debugging
# set -x

# Logging function
log() {
    echo "$(date +'%Y-%m-%d %H:%M:%S') - $*"
}

# Check if the required tools are installed
required_tools=("clang-format" "asmfmt")
for tool in "${required_tools[@]}"; do
    if ! command -v "$tool" &>/dev/null; then
        log "$tool is not installed. Aborting."
        exit 1
    fi
done

# Check if the .clang-format file exists in the current directory
if [ ! -f .clang-format ]; then
    log ".clang-format file not found in the base directory."
    exit 1
fi

# Function to convert // comments to /* ... */ comments in a file using awk
# Arguments:
#   $1: Path to the file to be processed
convert_comments() {
    local file_path=$1

    # Use awk to convert single-line comments to block comments,
    # ignoring lines with existing block comments or more than two slashes
    awk '
    {
        # Ignore lines with more than two slashes (e.g., ///)
        if ($0 ~ /\/\/\/+/) {
            print $0
        } 
        # Ignore lines with existing block comments (e.g., /* ... */)
        else if ($0 ~ /\/\*/) {
            print $0
        }
        # Convert single-line comments (//) to block comments (/* ... */) with proper spacing
        else if ($0 ~ /\/\// && $0 !~ /\".*\/\/.*\"/) {
            sub(/\/\//, "/*", $0)  # Replace // with /*
            sub(/[ \t]*$/, " */", $0)  # Add */ at the end, ensuring proper spacing
            # Ensure one space after /* if there were no spaces after //
            if ($0 ~ /\/\*[^ ]/) {
                sub(/\/\*/, "/* ", $0)  # Correct spacing
            }
            print $0
        } 
        # Print the line unchanged if none of the above conditions are met
        else {
            print $0
        }
    }
    ' "${file_path}" > "${file_path}.tmp" && mv "${file_path}.tmp" "${file_path}"
}

# Function to find and format files using a specified formatter
# Arguments:
#   $1: File extension to search for (e.g., "c", "h", "cc", "s")
#   $2: Formatter command to apply to each file
format_files() {
    local file_extension=$1
    local formatter_command=$2

    # Find all files with the specified extension, excluding those in the "build" directory
    # Process each file by converting comments and applying the formatter
    while IFS= read -r -d '' file; do
        log "Converting comments in $file"
        convert_comments "$file"
        log "Formatting $file"
        $formatter_command "$file"
    done < <(find . -type f ! -path "*/build/*" -name "*.${file_extension}" -print0)
}

# Export the functions so they can be used by the while loop
export -f convert_comments

# Define an array with the file extensions and their corresponding formatters
declare -a formatters=(
    "h:clang-format -i"
    "c:clang-format -i"
    "cc:clang-format -i"
    "s:asmfmt -w"
)

# Loop through the array and call format_files for each entry
for formatter in "${formatters[@]}"; do
    IFS=":" read -r extension command <<< "$formatter"
    format_files "$extension" "$command"
done

log "Formatting complete."
