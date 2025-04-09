#!/usr/bin/env bash
# display_files_markdown.sh
#
# This script finds and displays files matching a wide range of file types 
# commonly found in embedded projects with web interfaces.

# Default settings
SILENT_MODE=false
INCLUDE_TYPES=()
EXCLUDE_PATTERNS=()
OUTPUT_FILE=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --silent)
            SILENT_MODE=true
            shift
            ;;
        --include_type=*)
            # Split comma-separated types and add to INCLUDE_TYPES array
            IFS=',' read -ra TYPES <<< "${1#*=}"
            for type in "${TYPES[@]}"; do
                INCLUDE_TYPES+=("$(echo "$type" | tr '[:lower:]' '[:upper:]')")
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
        --output_file=*)
            OUTPUT_FILE="${1#*=}"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Find and display files with markdown formatting."
            echo ""
            echo "Options:"
            echo "  --include_type=TYPES    Comma-separated list of file types to include"
            echo "                          Multiple types can be specified: --include_type=C,H,CPP"
            echo "  --exclude=PATTERNS      Comma-separated list of files or paths to exclude"
            echo "                          Example: --exclude=vendor,tests,config.txt"
            echo ""
            echo "Language/Type Options:"
            echo "  C         - C source files (.c)"
            echo "  CPP       - C++ source files (.cpp, .cxx, .cc)"
            echo "  H         - C header files (.h)"
            echo "  HPP       - C++ header files (.hpp)"
            echo "  GO        - Go source files (.go)"
            echo "  PY        - Python source files (.py)"
            echo "  JS        - JavaScript files (.js, .mjs)"
            echo "  TS        - TypeScript files (.ts)"
            echo "  TSX       - TypeScript React files (.tsx)"
            echo "  HTML      - HTML files (.html)"
            echo "  CSS       - CSS files (.css)"
            echo "  VUE       - Vue files (.vue)"
            echo "  SVELTE    - Svelte files (.svelte)"
            echo "  SH        - Shell scripts (.sh)"
            echo "  PS1       - PowerShell scripts (.ps1)"
            echo "  CMAKE     - CMake files (CMakeLists.txt)"
            echo "  MAKE      - Makefiles (Makefile*)"
            echo "  KCONFIG   - Kconfig configuration files"
            echo "  JSON      - JSON files (.json)"
            echo "  YAML      - YAML files (.yaml, .yml)"
            echo "  TOML      - TOML files (.toml)"
            echo "  MD        - Markdown files (.md)"
            echo "  TXT       - Text files (.txt)"
            echo "  DOCKER    - Dockerfiles"
            echo "  ALL       - All supported file types"
            echo ""
            echo "Additional Options:"
            echo "  --silent           Silent mode: only copy to clipboard/file"
            echo "  --output_file=FILE Write output to specified file"
            echo ""
            echo "Examples:"
            echo "  $0 --include_type=C,H        # C and C header files"
            echo "  $0 --include_type=CPP,HPP    # C++ source and header files"
            echo "  $0 --include_type=ALL        # All file types"
            echo "  $0 --include_type=c,h        # Lowercase works too"
            echo "  $0 --exclude=config.txt,docs # Exclude specific file and directory"
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            echo "Use --help for usage information" >&2
            exit 1
            ;;
    esac
done

# If no include types specified, default to ALL
if [ ${#INCLUDE_TYPES[@]} -eq 0 ]; then
    INCLUDE_TYPES=("ALL")
fi

# Create find pattern based on specified types
build_find_pattern() {
    local pattern=""
    local separator=""

    for type in "${INCLUDE_TYPES[@]}"; do
        case "$type" in
            "C")
                pattern+="${separator}-name '*.c'"
                separator=" -o "
                ;;
            "CPP")
                pattern+="${separator}-name '*.cpp' -o -name '*.cxx' -o -name '*.cc'"
                separator=" -o "
                ;;
            "H")
                pattern+="${separator}-name '*.h'"
                separator=" -o "
                ;;
            "HPP")
                pattern+="${separator}-name '*.hpp'"
                separator=" -o "
                ;;
            "SH")
                pattern+="${separator}-name '*.sh'"
                separator=" -o "
                ;;
            "PS1")
                pattern+="${separator}-name '*.ps1'"
                separator=" -o "
                ;;
            "GO")
                pattern+="${separator}-name '*.go'"
                separator=" -o "
                ;;
            "PY")
                pattern+="${separator}-name '*.py'"
                separator=" -o "
                ;;
            "JS")
                pattern+="${separator}-name '*.js' -o -name '*.mjs'"
                separator=" -o "
                ;;
            "TS")
                pattern+="${separator}-name '*.ts'"
                separator=" -o "
                ;;
            "TSX")
                pattern+="${separator}-name '*.tsx'"
                separator=" -o "
                ;;
            "HTML")
                pattern+="${separator}-name '*.html'"
                separator=" -o "
                ;;
            "CSS")
                pattern+="${separator}-name '*.css'"
                separator=" -o "
                ;;
            "VUE")
                pattern+="${separator}-name '*.vue'"
                separator=" -o "
                ;;
            "SVELTE")
                pattern+="${separator}-name '*.svelte'"
                separator=" -o "
                ;;
            "CMAKE")
                pattern+="${separator}-name 'CMakeLists.txt'"
                separator=" -o "
                ;;
            "MAKE")
                pattern+="${separator}-name 'Makefile*'"
                separator=" -o "
                ;;
            "KCONFIG")
                pattern+="${separator}-name 'Kconfig' -o -name 'Kconfig.*'"
                separator=" -o "
                ;;
            "JSON")
                pattern+="${separator}-name '*.json'"
                separator=" -o "
                ;;
            "YAML")
                pattern+="${separator}-name '*.yaml' -o -name '*.yml'"
                separator=" -o "
                ;;
            "TOML")
                pattern+="${separator}-name '*.toml'"
                separator=" -o "
                ;;
            "MD")
                pattern+="${separator}-name '*.md'"
                separator=" -o "
                ;;
            "TXT")
                pattern+="${separator}-name '*.txt'"
                separator=" -o "
                ;;
            "DOCKER")
                pattern+="${separator}-name 'Dockerfile*'"
                separator=" -o "
                ;;
            "ALL")
                pattern="-name '*.c' -o -name '*.cpp' -o -name '*.cxx' -o -name '*.cc' \
                    -o -name '*.h' -o -name '*.hpp' \
                    -o -name '*.go' -o -name '*.py' \
                    -o -name '*.js' -o -name '*.mjs' -o -name '*.ts' -o -name '*.tsx' \
                    -o -name '*.html' -o -name '*.css' -o -name '*.vue' -o -name '*.svelte' \
                    -o -name '*.sh' -o -name '*.ps1' \
                    -o -name 'CMakeLists.txt' -o -name 'Makefile*' \
                    -o -name 'Kconfig' -o -name 'Kconfig.*' \
                    -o -name '*.json' -o -name '*.yaml' -o -name '*.yml' -o -name '*.toml' \
                    -o -name '*.md' -o -name '*.txt' -o -name 'Dockerfile*'"
                break
                ;;
            *)
                echo "Error: Invalid include type: $type" >&2
                echo "Valid types are: C, CPP, H, HPP, SH, PS1, GO, PY, JS, TS, TSX, HTML, CSS, VUE, SVELTE, CMAKE, MAKE, KCONFIG, JSON, YAML, TOML, MD, TXT, DOCKER, ALL" >&2
                exit 1
                ;;
        esac
    done

    echo "$pattern"
}

# Build find pattern
FIND_PATTERN=$(build_find_pattern)

# Function to process files:
process_files() {
    # Create a temporary exclude file pattern
    local exclude_file_pattern=""
    for exclude in "${EXCLUDE_PATTERNS[@]}"; do
        if [[ -n "$exclude_file_pattern" ]]; then
            exclude_file_pattern+="|"
        fi
        exclude_file_pattern+="$exclude"
    done
    
    # Default excluded directories
    local excluded_dirs="-path './build' -o -path './managed_components' -o -path './clang_format_backup_*' -o -path './node_modules' -o -path './dist'"
    
    # Use eval to correctly handle FIND_PATTERN with multiple -o options
    find_cmd="find . -type d \( $excluded_dirs \) -prune -o -type f \( $FIND_PATTERN \) -print"
    
    # Execute the find command and filter out excluded files
    eval "$find_cmd" | sort | while read -r file; do
        # Skip excluded files
        if [[ -n "$exclude_file_pattern" ]] && echo "$file" | grep -E "($exclude_file_pattern)" > /dev/null; then
            continue
        fi

        # Get the filename and determine language for markdown code block syntax.
        filename=$(basename "$file")
        case "$filename" in
            *.c)
                lang="c"
                ;;
            *.cpp|*.cxx|*.cc)
                lang="cpp"
                ;;
            *.h)
                lang="c"
                ;;
            *.hpp)
                lang="cpp"
                ;;
            *.sh)
                lang="bash"
                ;;
            *.ps1)
                lang="powershell"
                ;;
            *.go)
                lang="go"
                ;;
            *.py)
                lang="python"
                ;;
            *.js|*.mjs)
                lang="javascript"
                ;;
            *.ts)
                lang="typescript"
                ;;
            *.tsx)
                lang="typescript"
                ;;
            *.html)
                lang="html"
                ;;
            *.css)
                lang="css"
                ;;
            *.vue)
                lang="vue"
                ;;
            *.svelte)
                lang="svelte"
                ;;
            CMakeLists.txt)
                lang="cmake"
                ;;
            Makefile*)
                lang="makefile"
                ;;
            Kconfig*)
                lang="kconfig"
                ;;
            *.json)
                lang="json"
                ;;
            *.yaml|*.yml)
                lang="yaml"
                ;;
            *.toml)
                lang="toml"
                ;;
            *.md)
                lang="markdown"
                ;;
            *.txt)
                lang="text"
                ;;
            Dockerfile*)
                lang="dockerfile"
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
        # Only try to cat regular files to avoid "Is a directory" errors
        if [[ -f "$file" ]]; then
            cat "$file"
        else
            echo "[Not a regular file]"
        fi
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
