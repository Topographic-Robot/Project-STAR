#!/usr/bin/env bash
# display_files_markdown.sh
#
# This script finds and displays files matching specific patterns (.c, .h, CMakeLists.txt, Kconfig)
# with markdown formatting (triple backticks), excluding build, managed_components, 
# and clang_format_backup_* directories/files.
# In normal mode, the output is displayed in the terminal and also copied to the clipboard (on macOS).
# In silent mode (--silent), output is only copied to the clipboard without terminal display.
# Use --include_type option to filter files by type (C, H, CMake, Kconfig, or All).

# Default settings
SILENT_MODE=false
INCLUDE_TYPE="All"  # Default to including all file types

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
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Find and display files with markdown formatting."
            echo ""
            echo "Options:"
            echo "  --silent                 Silent mode: only copy to clipboard, no terminal output"
            echo "  --include_type=TYPE      Filter files by type. TYPE can be:"
            echo "                             C        - Include only .c files"
            echo "                             H        - Include only .h files"
            echo "                             CMake    - Include only CMakeLists.txt files"
            echo "                             Kconfig  - Include only Kconfig files"
            echo "                             All      - Include all file types (default)"
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

# Function to process files
process_files() {
    # Use eval to properly handle the FIND_PATTERN with multiple -o options
    eval "find . \
        -type d \( -path './build' -o -path './managed_components' -o -path './clang_format_backup_*' \) -prune -o \
        -type f \( $FIND_PATTERN \) \
        -not -path '*/clang_format_backup_*/*' \
        -print" | sort | while read -r file; do
        
        # Get file extension for language specification in markdown
        filename=$(basename "$file")
        extension="${filename##*.}"
        
        # Set language for markdown code block based on file type
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
        
        # Display file header and content with markdown formatting
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

# Execute based on mode
if $SILENT_MODE; then
    # Silent mode: only copy to clipboard
    if [ -n "$CLIPBOARD_CMD" ]; then
        process_files | eval "$CLIPBOARD_CMD"
        echo "Output has been copied to clipboard."
    else
        echo "Warning: No clipboard command available. Output will be displayed only." >&2
        process_files
    fi
else
    # Normal mode: display in terminal and copy to clipboard
    if [ -n "$CLIPBOARD_CMD" ]; then
        process_files | tee >(eval "$CLIPBOARD_CMD")
        echo "Output has been copied to clipboard."
    else
        echo "Warning: No clipboard command available. Output will be displayed only." >&2
        process_files
    fi
fi
