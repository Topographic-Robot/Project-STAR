#!/usr/bin/env bash
# display_files_markdown.sh
#
# This script finds and displays files matching specific patterns (.c, .h, CMakeLists.txt, Kconfig)
# with markdown formatting (triple backticks). It excludes build, managed_components, and
# clang_format_backup_* directories/files.
#
# In normal mode, the output is displayed in the terminal and also copied to the clipboard (on macOS,
# Linux with xclip, or Windows with clip). In silent mode (--silent), output is only copied to the clipboard
# (and/or saved to a file if --output_file is provided) without terminal display.
#
# Use --include_type option to filter files by type (C, H, CMake, Kconfig, or All).
# Use --output_file option to save the output to a specified file.
#
# Options:
#   --silent                 Silent mode: only copy to clipboard and/or save to file; no terminal output.
#   --include_type=TYPE      Filter files by type. TYPE can be:
#                                C        - Include only .c files.
#                                H        - Include only .h files.
#                                CMake    - Include only CMakeLists.txt files.
#                                Kconfig  - Include only Kconfig files.
#                                All      - Include all file types (default).
#   --output_file=FILE       Write the output to the specified file.
#   --help                   Display this help message and exit.
#
# Usage examples:
#   ./display_files_markdown.sh --include_type=C
#   ./display_files_markdown.sh --silent --output_file=output.md
#

# Default settings
SILENT_MODE=false
INCLUDE_TYPE="All"  # Default to including all file types
OUTPUT_FILE=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --silent)
            SILENT_MODE=true
            shift
            ;;
        --include_type=*)
            INCLUDE_TYPE="${1#*=}"
            shift
            ;;
        --output_file=*)
            OUTPUT_FILE="${1#*=}"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Find and display files with markdown formatting."
            echo ""
            echo "Options:"
            echo "  --silent                 Silent mode: only copy to clipboard and/or save to file; no terminal output"
            echo "  --include_type=TYPE      Filter files by type. TYPE can be:"
            echo "                             C        - Include only .c files"
            echo "                             H        - Include only .h files"
            echo "                             CMake    - Include only CMakeLists.txt files"
            echo "                             Kconfig  - Include only Kconfig files"
            echo "                             All      - Include all file types (default)"
            echo "  --output_file=FILE       Write the output to the specified file"
            echo "  --help                   Display this help message and exit"
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            echo "Use --help for usage information" >&2
            exit 1
            ;;
    esac
done

# Normalize INCLUDE_TYPE to uppercase for case-insensitive comparison
INCLUDE_TYPE=$(echo "$INCLUDE_TYPE" | tr '[:lower:]' '[:upper:]')

# Set the find command pattern based on the include type
FIND_PATTERN=""
case "$INCLUDE_TYPE" in
    "C")
        FIND_PATTERN="-name '*.c'"
        ;;
    "H")
        FIND_PATTERN="-name '*.h'"
        ;;
    "CMAKE")
        FIND_PATTERN="-name 'CMakeLists.txt'"
        ;;
    "KCONFIG")
        FIND_PATTERN="-name 'Kconfig' -o -name 'Kconfig.*'"
        ;;
    "ALL")
        FIND_PATTERN="-name 'CMakeLists.txt' -o -name 'Kconfig' -o -name 'Kconfig.*' -o -name '*.c' -o -name '*.h'"
        ;;
    *)
        echo "Error: Invalid include type: $INCLUDE_TYPE" >&2
        echo "Valid types are: C, H, CMake, Kconfig, All" >&2
        exit 1
        ;;
esac

# Function to process files:
process_files() {
    # Use eval to correctly handle FIND_PATTERN with multiple -o options.
    eval "find . \
        -type d \( -path './build' -o -path './managed_components' -o -path './clang_format_backup_*' \) -prune -o \
        -type f \( $FIND_PATTERN \) \
        -not -path '*/clang_format_backup_*/*' \
        -print" | sort | while read -r file; do

        # Get the filename and determine language for markdown code block syntax.
        filename=$(basename "$file")
        case "$filename" in
            *.c)
                lang="c"
                ;;
            *.h)
                lang="c"
                ;;
            CMakeLists.txt)
                lang="cmake"
                ;;
            Kconfig*)
                lang="kconfig"
                ;;
            *)
                lang=""
                ;;
        esac

        # Output file header and contents in markdown format.
        echo
        echo "# $file"
        echo
        echo "\`\`\`$lang"
        cat "$file"
        echo "\`\`\`"
        echo
    done
}

# Determine clipboard command based on operating system
CLIPBOARD_CMD=""
if command -v pbcopy &> /dev/null; then
    # macOS
    CLIPBOARD_CMD="pbcopy"
elif command -v xclip &> /dev/null; then
    # Linux with xclip
    CLIPBOARD_CMD="xclip -selection clipboard"
elif command -v clip &> /dev/null; then
    # Windows
    CLIPBOARD_CMD="clip"
fi

#
# Execute based on mode (normal or silent) and destination (terminal, clipboard, file)
#
if $SILENT_MODE; then
    # In silent mode, do not display to terminal.
    if [ -n "$CLIPBOARD_CMD" ] && [ -n "$OUTPUT_FILE" ]; then
        process_files | tee "$OUTPUT_FILE" | eval "$CLIPBOARD_CMD" > /dev/null
        echo "Output has been copied to clipboard and saved to file: $OUTPUT_FILE." >&2
    elif [ -n "$CLIPBOARD_CMD" ]; then
        process_files | eval "$CLIPBOARD_CMD" > /dev/null
        echo "Output has been copied to clipboard." >&2
    elif [ -n "$OUTPUT_FILE" ]; then
        process_files > "$OUTPUT_FILE"
        echo "Output has been saved to file: $OUTPUT_FILE." >&2
    else
        echo "Warning: No clipboard command or output file specified. Output will be displayed." >&2
        process_files
    fi
else
    # In normal mode, display output in terminal.
    if [ -n "$CLIPBOARD_CMD" ] && [ -n "$OUTPUT_FILE" ]; then
        process_files | tee >(eval "$CLIPBOARD_CMD") | tee "$OUTPUT_FILE"
        echo "Output has been copied to clipboard and saved to file: $OUTPUT_FILE."
    elif [ -n "$CLIPBOARD_CMD" ]; then
        process_files | tee >(eval "$CLIPBOARD_CMD")
        echo "Output has been copied to clipboard."
    elif [ -n "$OUTPUT_FILE" ]; then
        process_files | tee "$OUTPUT_FILE"
        echo "Output has been saved to file: $OUTPUT_FILE."
    else
        process_files
    fi
fi

