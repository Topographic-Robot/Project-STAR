#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status,
# treat unset variables as an error,
# and ensure that the return value of a pipeline is the status of the last command to exit with a non-zero status
set -euo pipefail

# Uncomment the following line for debugging
# set -x

# Logging function to print messages with a timestamp
log() {
    echo "$(date +'%Y-%m-%d %H:%M:%S') - $*"
}

# Check if the required tools are installed
required_tools=("clang-format" "asmfmt" "dos2unix")
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
    # ignoring lines with existing block comments or more than two slashes,
    # and properly handling quoted strings
    awk '
    {
        # Split the line by double quotes (")
        n = split($0, parts, /"/)

        # Flag to track whether we are inside a quoted string
        inside_quote = 0

        # Iterate through each part split by quotes
        for (i = 1; i <= n; i++) {
            # If we are not inside a quoted string
            if (inside_quote == 0) {
                # Check if the part contains a // comment and does not contain a block comment (/*)
                if (parts[i] ~ /\/\// && parts[i] !~ /\/\*/) {
                    # Replace // with /* and add space if necessary
                    sub(/\/\//, "/*", parts[i])
                    # Add */ at the end, ensuring proper spacing
                    sub(/[ \t]*$/, " */", parts[i])
                }

                # Ensure there is exactly one space after /*
                gsub(/\/\* */, "/* ", parts[i])
                gsub(/\/\*\*/, "/* ", parts[i])
                # Ensure there is exactly one space before */
                gsub(/ *\*\//, " */", parts[i])
            }
            inside_quote = 1 - inside_quote # Toggle the inside_quote flag to indicate entering/exiting a quoted string
        }

        # Reconstruct the line by concatenating the parts with double quotes
        $0 = parts[1]
        for (i = 2; i <= n; i++) {
            $0 = $0 "\"" parts[i]
        }
        print
    }
    ' "${file_path}" > "${file_path}.tmp" && mv "${file_path}.tmp" "${file_path}"
}

# Function to convert DOS line endings to Unix line endings
# Arguments:
#   $1: Path to the file to be processed
convert_dos_to_unix() {
    local file_path=$1
    dos2unix "$file_path"
}

# Function to find and format files using a specified formatter
# Arguments:
#   $1: File extension to search for (e.g., "c", "h", "cc", "s")
#   $2: Formatter command to apply to each file
#   $3: Backup directory to use for backups
format_files() {
    local file_extension=$1
    local formatter_command=$2
    local backup_dir=$3

    # Create the backup directory
    mkdir -p "$backup_dir"

    # Find all files with the specified extension, excluding those in the "build" and "backup_" directories
    # Process each file by converting DOS to Unix, converting comments, and applying the formatter
    find . -type f ! -path "*/build/*" ! -path "*/backup_*/*" -name "*.${file_extension}" -print0 | while IFS= read -r -d '' file; do
        # Create backup of the original file
        local backup_file="$backup_dir/$file"
        mkdir -p "$(dirname "$backup_file")"
        cp "$file" "$backup_file"

        # Log the conversion and formatting steps
        log "Converting DOS to Unix line endings in $file"
        convert_dos_to_unix "$file"
        log "Converting comments in $file"
        convert_comments "$file"
        log "Formatting $file"
        $formatter_command "$file"

        # Check if the backup differs from the formatted file
        if ! diff -q "$file" "$backup_file" &>/dev/null; then
            log "Backup differs for $file. Keeping backup."
        else
            log "No changes detected for $file. Removing backup."
            rm "$backup_file"
            # Remove empty backup directories
            find "$backup_dir" -type d -empty -delete
        fi
    done
}

# Export the functions so they can be used by the while loop
export -f convert_comments
export -f convert_dos_to_unix
export -f format_files

# Define the backup directory with a timestamp
backup_dir="backup/backup_$(date +'%Y%m%d%H%M%S')"

# Define an array with the file extensions and their corresponding formatters
declare -a formatters=(
    "h:clang-format -i"
    "c:clang-format -i"
    "cc:clang-format -i"
    "cpp:clang-format -i"
    "s:asmfmt -w"
)

# Loop through the array and call format_files for each entry
for formatter in "${formatters[@]}"; do
    IFS=":" read -r extension command <<< "$formatter"
    format_files "$extension" "$command" "$backup_dir"
done

# Remove the backup directory if it is empty
if [ -d "$backup_dir" ]; then
    find "$backup_dir" -type d -empty -delete
    if [ -d "$backup_dir" ] && [ ! "$(ls -A "$backup_dir")" ]; then
        rmdir "$backup_dir"
        rmdir "backup"
    fi
fi

log "Formatting complete."
