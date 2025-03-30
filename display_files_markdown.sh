#!/usr/bin/env bash
# display_files_markdown.sh
#
# This script finds and displays files matching specific patterns (.c, .h, CMakeLists.txt, Kconfig)
# with markdown formatting (triple backticks), excluding build and managed_components directories.
# In normal mode, the output is displayed in the terminal and also copied to the clipboard (on macOS).
# In silent mode (--silent), output is only copied to the clipboard without terminal display.

# Default settings
SILENT_MODE=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --silent)
            SILENT_MODE=true
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Find and display files with markdown formatting."
            echo ""
            echo "Options:"
            echo "  --silent    Silent mode: only copy to clipboard, no terminal output"
            echo "  --help      Display this help message and exit"
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            echo "Use --help for usage information" >&2
            exit 1
            ;;
    esac
done

# Function to process files
process_files() {
    find . \( -path './build' -o -path './managed_components' \) -prune -o \( -type f \( -name 'CMakeLists.txt' -o -name 'Kconfig' -o -name 'Kconfig.*' -o -name '*.c' -o -name '*.h' \) \) -print | while read -r file; do
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

# Check if pbcopy is available (macOS)
if ! command -v pbcopy &> /dev/null; then
    echo "Warning: pbcopy is not available. Clipboard functionality will not work." >&2
    SILENT_MODE=false  # Force non-silent mode if pbcopy isn't available
fi

# Execute based on mode
if $SILENT_MODE; then
    # Silent mode: only copy to clipboard
    process_files | pbcopy
    echo "Output has been copied to clipboard."
else
    # Normal mode: display in terminal and copy to clipboard
    process_files | tee >(pbcopy)
    echo "Output has been copied to clipboard."
fi
