#!/usr/bin/env bash
# display_files_markdown.sh
#
# This script finds and displays files matching a wide range of file types
# commonly found in embedded projects with web interfaces.
# It formats output in markdown syntax for easy integration with documentation tools.

# Default settings
SILENT_MODE=false
INCLUDE_TYPES=()
EXCLUDE_PATTERNS=()
INCLUDE_FILES=()
OUTPUT_FILE=""
VERBOSE=false
RECURSIVE=true
MAX_DEPTH=""
VERSION="1.1.0"
SEARCH_DIR="."  # Default to current directory
USE_CLIPBOARD=true  # Default to using clipboard when available

# Function to get file extensions for a type
get_file_extensions() {
    local type="$1"
    case "$type" in
        "C") echo "*.c" ;;
        "CPP") echo "*.cpp *.cxx *.cc" ;;
        "H") echo "*.h" ;;
        "HPP") echo "*.hpp" ;;
        "GO") echo "*.go" ;;
        "PY") echo "*.py" ;;
        "JS") echo "*.js *.mjs" ;;
        "TS") echo "*.ts" ;;
        "TSX") echo "*.tsx" ;;
        "HTML") echo "*.html" ;;
        "CSS") echo "*.css" ;;
        "VUE") echo "*.vue" ;;
        "SVELTE") echo "*.svelte" ;;
        "SH") echo "*.sh" ;;
        "PS1") echo "*.ps1" ;;
        "CMAKE") echo "CMakeLists.txt" ;;
        "MAKE") echo "Makefile*" ;;
        "KCONFIG") echo "Kconfig Kconfig.*" ;;
        "JSON") echo "*.json" ;;
        "YAML") echo "*.yaml *.yml" ;;
        "TOML") echo "*.toml" ;;
        "MD") echo "*.md" ;;
        "TXT") echo "*.txt" ;;
        "GD") echo "*.gd" ;;
        "DOCKER") echo "Dockerfile*" ;;
        *) echo "" ;;
    esac
}

# Function to get language from filename for markdown highlighting
get_language_from_filename() {
    local filename="$1"
    local ext="${filename##*.}"
    local basename=$(basename "$filename")

    case "$basename" in
        *.c)
            echo "c"
            ;;
        *.cpp|*.cxx|*.cc)
            echo "cpp"
            ;;
        *.h)
            echo "c"
            ;;
        *.hpp)
            echo "cpp"
            ;;
        *.sh)
            echo "bash"
            ;;
        *.ps1)
            echo "powershell"
            ;;
        *.go)
            echo "go"
            ;;
        *.py)
            echo "python"
            ;;
        *.js|*.mjs)
            echo "javascript"
            ;;
        *.ts)
            echo "typescript"
            ;;
        *.tsx)
            echo "typescript"
            ;;
        *.html)
            echo "html"
            ;;
        *.css)
            echo "css"
            ;;
        *.vue)
            echo "vue"
            ;;
        *.svelte)
            echo "svelte"
            ;;
        CMakeLists.txt)
            echo "cmake"
            ;;
        Makefile*)
            echo "makefile"
            ;;
        Kconfig*)
            echo "kconfig"
            ;;
        *.json)
            echo "json"
            ;;
        *.yaml|*.yml)
            echo "yaml"
            ;;
        *.toml)
            echo "toml"
            ;;
        *.md)
            echo "markdown"
            ;;
        *.txt)
            echo "text"
            ;;
        *.gd)
            echo "gdscript"
            ;;
        Dockerfile*)
            echo "dockerfile"
            ;;
        *)
            echo ""
            ;;
    esac
}

# Get all supported file types
get_all_file_types() {
    echo "C CPP H HPP GO PY JS TS TSX HTML CSS VUE SVELTE SH PS1 CMAKE MAKE KCONFIG JSON YAML TOML MD TXT GD DOCKER"
}

# Default excluded directories
DEFAULT_EXCLUDED_DIRS=(
    "build"
    "managed_components"
    "clang_format_backup_*"
    "node_modules"
    "dist"
    ".git"
    "vendor"
)

# Print help message
show_help() {
    cat << EOF
Usage: $0 [OPTIONS]
Find and display files with markdown formatting.

Version: $VERSION

Options:
  --include_type=TYPES    Comma-separated list of file types to include
                          Multiple types can be specified: --include_type=C,H,CPP
  --include_files=FILES   Comma-separated list of specific files to include
                          Example: --include_files=src/main.c,include/config.h
  --exclude=PATTERNS      Comma-separated list of files or paths to exclude
                          Example: --exclude=vendor,tests,config.txt
  --output_file=FILE      Write output to specified file
  --dir=DIRECTORY         Specify the root directory to search (default: current directory)
  --silent                Silent mode: only copy to clipboard/file
  --no-clipboard          Disable copying to clipboard
  --non-recursive         Do not search recursively in directories
  --max-depth=N           Search only N levels deep in directory structure
  --verbose               Display additional processing information
  --version               Show version information
  --help                  Display this help message

Language/Type Options:
EOF
    # Sort and display available file types
    for type in $(get_all_file_types | tr ' ' '\n' | sort); do
        extensions=$(get_file_extensions "$type")
        if [ "$type" == "ALL" ]; then
            echo "  ALL       - All supported file types"
        else
            echo "  $type$(printf '%*s' $((10-${#type})) '') - $(echo "$extensions" | sed 's/ /, /g')"
        fi
    done

    cat << EOF

Additional Notes:
  - If no types or files are specified, ALL types are included by default
  - Output is copied to clipboard when available unless --no-clipboard is used

Examples:
  $0 --include_type=C,H                      # C and C header files
  $0 --include_files=main.c,config.h         # Only specific files
  $0 --include_type=CPP --include_files=main.c  # Combine types and files
  $0 --include_type=ALL                      # All file types
  $0 --exclude=config.txt,docs               # Exclude specific file and directory
  $0 --max-depth=2                           # Limit directory search depth
  $0 --dir=/path/to/project                  # Search in a specific directory
  $0 --no-clipboard                          # Don't copy output to clipboard
EOF
}

# Function to detect clipboard command
detect_clipboard_cmd() {
    if [ "$USE_CLIPBOARD" = false ]; then
        echo ""
        return
    fi

    if command -v pbcopy &> /dev/null; then
        # macOS
        echo "pbcopy"
    elif command -v xclip &> /dev/null; then
        # Linux with xclip
        echo "xclip -selection clipboard"
    elif command -v xsel &> /dev/null; then
        # Linux with xsel
        echo "xsel --clipboard --input"
    elif command -v clip &> /dev/null; then
        # Windows
        echo "clip"
    else
        echo ""
    fi
}

# Log function for verbose output
log() {
    if [ "$VERBOSE" = true ]; then
        echo "[INFO] $1" >&2
    fi
}

# Error logging
error() {
    echo "[ERROR] $1" >&2
}

# Warning logging
warning() {
    echo "[WARNING] $1" >&2
}

# Check if a type is valid
is_valid_type() {
    local type="$1"
    local all_types=$(get_all_file_types)
    local found=false

    if [ "$type" = "ALL" ]; then
        return 0
    fi

    for t in $all_types; do
        if [ "$t" = "$type" ]; then
            return 0
        fi
    done

    return 1
}

# Parse command line arguments
parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --silent)
                SILENT_MODE=true
                shift
                ;;
            --verbose)
                VERBOSE=true
                shift
                ;;
            --non-recursive)
                RECURSIVE=false
                shift
                ;;
            --no-clipboard)
                USE_CLIPBOARD=false
                shift
                ;;
            --max-depth=*)
                MAX_DEPTH="${1#*=}"
                # Validate max depth is a number
                if ! [[ "$MAX_DEPTH" =~ ^[0-9]+$ ]]; then
                    error "Max depth must be a number"
                    exit 1
                fi
                shift
                ;;
            --dir=*)
                SEARCH_DIR="${1#*=}"
                # Check if directory exists
                if [ ! -d "$SEARCH_DIR" ]; then
                    error "Directory does not exist: $SEARCH_DIR"
                    exit 1
                fi
                shift
                ;;
            --include_type=*|--include-type=*)
                # Split comma-separated types and add to INCLUDE_TYPES array
                IFS=',' read -ra TYPES <<< "${1#*=}"
                for type in "${TYPES[@]}"; do
                    type_upper=$(echo "$type" | tr '[:lower:]' '[:upper:]')
                    if is_valid_type "$type_upper"; then
                        INCLUDE_TYPES+=("$type_upper")
                    else
                        error "Invalid include type: $type"
                        echo "Valid types are: $(get_all_file_types)" >&2
                        exit 1
                    fi
                done
                shift
                ;;
            --include_files=*|--include-files=*)
                # Split comma-separated files and add to INCLUDE_FILES array
                IFS=',' read -ra FILES <<< "${1#*=}"
                for file in "${FILES[@]}"; do
                    INCLUDE_FILES+=("$file")
                done
                shift
                ;;
            --exclude=*)
                # Split comma-separated patterns and add to EXCLUDE_PATTERNS array
                IFS=',' read -ra PATTERNS <<< "${1#*=}"
                for pattern in "${PATTERNS[@]}"; do
                    EXCLUDE_PATTERNS+=("$pattern")
                done
                shift
                ;;
            --output_file=*|--output-file=*)
                OUTPUT_FILE="${1#*=}"
                shift
                ;;
            --version)
                echo "display_files_markdown.sh version $VERSION"
                exit 0
                ;;
            --help)
                show_help
                exit 0
                ;;
            *)
                error "Unknown option: $1"
                echo "Use --help for usage information" >&2
                exit 1
                ;;
        esac
    done

    # If no include types and no include files specified, default to ALL
    if [ ${#INCLUDE_TYPES[@]} -eq 0 ] && [ ${#INCLUDE_FILES[@]} -eq 0 ]; then
        log "No file types or files specified, defaulting to ALL types"
        INCLUDE_TYPES=("ALL")
    fi

    log "Search directory: $SEARCH_DIR"
    if [ "$USE_CLIPBOARD" = false ]; then
        log "Clipboard usage disabled"
    fi
}

# Create find pattern based on specified types
build_find_pattern() {
    local pattern=""
    local separator=""

    # Handle the ALL type specially
    if echo "${INCLUDE_TYPES[@]}" | grep -q "ALL"; then
        for type in $(get_all_file_types); do
            for ext in $(get_file_extensions "$type"); do
                pattern+="${separator}-name '$ext'"
                separator=" -o "
            done
        done
        echo "$pattern"
        return
    fi

    # Handle specific types
    for type in "${INCLUDE_TYPES[@]}"; do
        for ext in $(get_file_extensions "$type"); do
            pattern+="${separator}-name '$ext'"
            separator=" -o "
        done
    done

    echo "$pattern"
}

# Build the exclude pattern for find command
build_exclude_pattern() {
    local excluded_dirs_pattern=""
    local separator=""

    # Add default excluded directories
    for dir in "${DEFAULT_EXCLUDED_DIRS[@]}"; do
        excluded_dirs_pattern+="${separator}-path '*/$dir' -o -path '$dir'"
        separator=" -o "
    done

    # Add user-specified exclude patterns that look like directories
    for pattern in "${EXCLUDE_PATTERNS[@]}"; do
        if [[ "$pattern" == */ || -d "$SEARCH_DIR/$pattern" ]]; then
            excluded_dirs_pattern+="${separator}-path '*/$pattern' -o -path '$pattern'"
            separator=" -o "
        fi
    done

    echo "$excluded_dirs_pattern"
}

# Function to format and output a file
format_file() {
    local file="$1"
    local filename=$(basename "$file")
    local lang=$(get_language_from_filename "$filename")

    # Skip if file doesn't exist anymore (could have been deleted during processing)
    if [[ ! -e "$file" ]]; then
        warning "File no longer exists: $file"
        return
    fi

    # Skip directories
    if [[ -d "$file" ]]; then
        log "Skipping directory: $file"
        return
    fi

    log "Processing file: $file"

    # Output file header and contents in markdown format
    echo
    echo "# $file"
    echo
    echo "\`\`\`$lang"
    # Only try to cat regular files to avoid "Is a directory" errors
    if [[ -f "$file" ]]; then
        # Check if file is binary
        if file "$file" | grep -q "binary"; then
            echo "[Binary file - contents not displayed]"
        else
            cat "$file" || echo "[Error reading file]"
        fi
    else
        echo "[Not a regular file]"
    fi
    echo "\`\`\`"
    echo
}

# Process explicitly included files
process_included_files() {
    local exclude_regex=""

    # Build exclude regex if needed
    if [ ${#EXCLUDE_PATTERNS[@]} -gt 0 ]; then
        exclude_regex=$(IFS="|"; echo "${EXCLUDE_PATTERNS[*]}")
    fi

    log "Processing ${#INCLUDE_FILES[@]} explicitly included files"

    for file in "${INCLUDE_FILES[@]}"; do
        local full_path
        # Check if the path is absolute or relative
        if [[ "$file" == /* ]]; then
            full_path="$file"
        else
            full_path="$SEARCH_DIR/$file"
        fi

        # Skip excluded files
        if [[ -n "$exclude_regex" ]] && echo "$file" | grep -E "($exclude_regex)" > /dev/null; then
            log "Skipping excluded file: $file"
            continue
        fi

        # Check if file exists
        if [[ -f "$full_path" ]]; then
            format_file "$full_path"
        else
            warning "File not found: $full_path"
        fi
    done
}

# Function to check if a file is in the array of included files
is_in_included_files() {
    local file="$1"
    local normalized_file="${file#$SEARCH_DIR/}"

    for included_file in "${INCLUDE_FILES[@]}"; do
        if [ "$normalized_file" = "$included_file" ] || [ "$file" = "$included_file" ]; then
            return 0
        fi
    done

    return 1
}

# Function to process files from find command
process_found_files() {
    local find_pattern="$1"
    local exclude_dirs_pattern="$2"
    local exclude_regex=""

    # Build exclude regex if needed
    if [ ${#EXCLUDE_PATTERNS[@]} -gt 0 ]; then
        exclude_regex=$(IFS="|"; echo "${EXCLUDE_PATTERNS[*]}")
    fi

    # Build find command with proper syntax
    local find_cmd="find \"$SEARCH_DIR\" "

    # Add depth limit if specified
    if [ "$RECURSIVE" = false ]; then
        find_cmd+=" -maxdepth 1 "
    elif [ -n "$MAX_DEPTH" ]; then
        find_cmd+=" -maxdepth $MAX_DEPTH "
    fi

    # Add directory exclusions
    if [ -n "$exclude_dirs_pattern" ]; then
        find_cmd+="-type d \\( $exclude_dirs_pattern \\) -prune -o "
    fi

    # Add file pattern - fixed the syntax here
    find_cmd+="-type f \\( $find_pattern \\) -print"

    log "Executing find command: $find_cmd"

    # Execute the find command and filter out excluded files
    eval "$find_cmd" | sort | while read -r file; do
        # Skip explicitly included files (they were already processed)
        if is_in_included_files "$file"; then
            log "Skipping already processed file: $file"
            continue
        fi

        # Skip excluded files
        if [[ -n "$exclude_regex" ]] && echo "$file" | grep -E "($exclude_regex)" > /dev/null; then
            log "Skipping excluded file: $file"
            continue
        fi

        format_file "$file"
    done
}

# Function to process all files
process_files() {
    # First process explicitly included files
    if [ ${#INCLUDE_FILES[@]} -gt 0 ]; then
        process_included_files
    fi

    # Then process files by type if any types specified
    if [ ${#INCLUDE_TYPES[@]} -gt 0 ]; then
        # Build find pattern
        local find_pattern=$(build_find_pattern)
        local exclude_dirs_pattern=$(build_exclude_pattern)

        log "File pattern: $find_pattern"
        log "Exclude directories pattern: $exclude_dirs_pattern"

        process_found_files "$find_pattern" "$exclude_dirs_pattern"
    fi
}

# Function to output results based on mode and destination
output_results() {
    # Determine clipboard command
    local clipboard_cmd=$(detect_clipboard_cmd)

    if [ "$SILENT_MODE" = true ]; then
        # In silent mode, do not display to terminal
        if [ -n "$clipboard_cmd" ] && [ -n "$OUTPUT_FILE" ]; then
            process_files | tee "$OUTPUT_FILE" | eval "$clipboard_cmd" > /dev/null
            echo "Output has been copied to clipboard and saved to file: $OUTPUT_FILE." >&2
        elif [ -n "$clipboard_cmd" ]; then
            process_files | eval "$clipboard_cmd" > /dev/null
            echo "Output has been copied to clipboard." >&2
        elif [ -n "$OUTPUT_FILE" ]; then
            process_files > "$OUTPUT_FILE"
            echo "Output has been saved to file: $OUTPUT_FILE." >&2
        else
            warning "No clipboard command or output file specified. Output will be displayed."
            process_files
        fi
    else
        # In normal mode, display output in terminal
        if [ -n "$clipboard_cmd" ] && [ -n "$OUTPUT_FILE" ]; then
            process_files | tee >(eval "$clipboard_cmd") | tee "$OUTPUT_FILE"
            echo "Output has been copied to clipboard and saved to file: $OUTPUT_FILE."
        elif [ -n "$clipboard_cmd" ]; then
            process_files | tee >(eval "$clipboard_cmd")
            echo "Output has been copied to clipboard."
        elif [ -n "$OUTPUT_FILE" ]; then
            process_files | tee "$OUTPUT_FILE"
            echo "Output has been saved to file: $OUTPUT_FILE."
        else
            process_files
        fi
    fi
}

# Main function
main() {
    parse_arguments "$@"
    output_results
}

# Execute main function with all arguments
main "$@"
