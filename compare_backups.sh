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

# Function to run diff on each file in the backup folder against the corresponding file in the original folder
# Arguments:
#   $1: Path to the backup folder
#   $2: Path to the original folder
compare_files() {
    local backup_folder=$1
    local original_folder=$2

    while IFS= read -r -d '' backup_file; do
        relative_path="${backup_file#$backup_folder/}"
        relative_path="${relative_path#*/}" # Remove the leading directory (e.g., "basics")
        original_file="${original_folder}/${relative_path}"

        if [ -f "$original_file" ]; then
            log "Comparing $backup_file with $original_file"
            if ! diff -q "$backup_file" "$original_file" >/dev/null; then
                log "Differences found:"
                diff "$backup_file" "$original_file"
            else
                log "No differences found."
            fi
        else
            log "Original file $original_file does not exist"
        fi
    done < <(find "$backup_folder" -type f -print0)
}

# Main function to validate input and call the compare_files function
main() {
    if [ $# -ne 2 ]; then
        echo "Usage: $0 <backup_folder> <original_folder>"
        exit 1
    fi

    local backup_folder=$1
    local original_folder=$2

    if [ ! -d "$backup_folder" ]; then
        echo "Backup folder $backup_folder does not exist"
        exit 1
    fi

    if [ ! -d "$original_folder" ]; then
        echo "Original folder $original_folder does not exist"
        exit 1
    fi

    compare_files "$backup_folder" "$original_folder"
}

main "$@"
