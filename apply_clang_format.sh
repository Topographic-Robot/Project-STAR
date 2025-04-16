#!/usr/bin/env bash
# apply_clang_format.sh
#
# This script applies clang-format to C/C++ source files (.c, .cpp, .h, .hpp)
# using the .clang-format file in the project root. It can operate in several modes:
#   1. Check-only: Reports files that would be modified without making changes
#   2. Fix: Reformats files according to the .clang-format configuration
#   3. Specific directory: Process only files in the specified directory
#   4. Purge: Remove all existing clang-format backup directories
#
# The script excludes 'build', 'managed_components', and backup directories by default.
# Includes safety checks to prevent file corruption.

# Default settings
CHECK_ONLY=false
FILE_TYPES=("*.c" "*.cpp" "*.h" "*.hpp")
EXCLUDE_DIRS=("build" "managed_components")
BACKUP_DIR="clang_format_backup_$(date +%Y%m%d%H%M%S)"
VALIDATE_CONFIG=true
TARGET_DIR="."
INTERACTIVE=true
SIZE_THRESHOLD=75  # Files reduced to less than 75% of original size will be flagged (percentage as integer)
PURGE_BACKUPS=false
BACKUP_CREATED=false

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Ensure clang-format is available
if ! command -v clang-format &> /dev/null; then
    echo -e "${RED}Error: clang-format is not installed. Please install it and try again.${NC}" >&2
    exit 1
fi

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --check)
            CHECK_ONLY=true
            shift
            ;;
        --fix)
            CHECK_ONLY=false
            shift
            ;;
        --dir=*)
            TARGET_DIR="${1#*=}"
            shift
            ;;
        --no-validate)
            VALIDATE_CONFIG=false
            shift
            ;;
        --non-interactive)
            INTERACTIVE=false
            shift
            ;;
        --exclude=*)
            additional_exclude="${1#*=}"
            EXCLUDE_DIRS+=("$additional_exclude")
            shift
            ;;
        --threshold=*)
            threshold_value="${1#*=}"
            # Convert decimal to integer percentage
            if [[ "$threshold_value" =~ ^0\.[0-9]+$ ]]; then
                # Convert decimal (like 0.75) to percentage (75)
                SIZE_THRESHOLD=$(echo "$threshold_value * 100" | bc | cut -d. -f1)
            elif [[ "$threshold_value" =~ ^[0-9]+$ ]]; then
                # Already an integer percentage
                SIZE_THRESHOLD=$threshold_value
            else
                echo -e "${RED}Error: Invalid threshold value. Use decimal (0.75) or percentage (75).${NC}" >&2
                exit 1
            fi
            shift
            ;;
        --purge-backups)
            PURGE_BACKUPS=true
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Apply clang-format to C/C++ source files using the project's .clang-format file."
            echo ""
            echo "Options:"
            echo "  --check              Only check and report files that would be modified (default: false)"
            echo "  --fix                Fix files by applying clang-format (default action)"
            echo "  --dir=PATH           Process only files in the specified directory"
            echo "  --exclude=DIR        Exclude additional directory from processing"
            echo "  --no-validate        Skip validation of .clang-format file"
            echo "  --non-interactive    Run without interactive prompts"
            echo "  --threshold=NUM      Set size reduction threshold (default: 75, meaning 75%)"
            echo "                       Can be specified as decimal (0.75) or percentage (75)"
            echo "  --purge-backups      Remove all existing clang-format backup directories"
            echo "  --help               Display this help message and exit"
            echo ""
            echo "Examples:"
            echo "  $0 --check                  Check all files without modifying them"
            echo "  $0 --dir=components/mylib   Format files in components/mylib directory"
            echo "  $0 --exclude=tests          Exclude the tests directory"
            echo "  $0 --threshold=60           Allow up to 40% size reduction before warning"
            echo "  $0 --purge-backups          Remove all existing backup directories"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}" >&2
            echo "Use --help for usage information" >&2
            exit 1
            ;;
    esac
done

# Handle purge backups option
if $PURGE_BACKUPS; then
    echo "Looking for clang-format backup directories to purge..."
    backup_dirs=$(find . -maxdepth 1 -type d -name "clang_format_backup_*" 2>/dev/null)

    if [[ -z "$backup_dirs" ]]; then
        echo -e "${YELLOW}No clang-format backup directories found.${NC}"
    else
        backup_count=$(echo "$backup_dirs" | wc -l)
        echo -e "${YELLOW}Found $backup_count backup directories to remove.${NC}"

        if $INTERACTIVE; then
            echo "$backup_dirs"
            read -p "Are you sure you want to remove these directories? [y/N] " confirm
            if [[ "$confirm" =~ ^[Yy]$ ]]; then
                echo "Removing backup directories..."
                echo "$backup_dirs" | xargs rm -rf
                echo -e "${GREEN}Successfully removed $backup_count backup directories.${NC}"
            else
                echo "Operation cancelled."
                exit 0
            fi
        else
            echo "Removing backup directories..."
            echo "$backup_dirs" | xargs rm -rf
            echo -e "${GREEN}Successfully removed $backup_count backup directories.${NC}"
        fi

        # Exit if purge was the only operation requested
        if $CHECK_ONLY && [ "$TARGET_DIR" = "." ]; then
            exit 0
        fi
    fi
fi

# Check for diff tools for interactive comparison
DIFF_TOOL=""
if $INTERACTIVE; then
    # Check for graphical diff tools in order of preference
    if command -v code &> /dev/null; then
        DIFF_TOOL="code --diff"
    elif command -v meld &> /dev/null; then
        DIFF_TOOL="meld"
    elif command -v kdiff3 &> /dev/null; then
        DIFF_TOOL="kdiff3"
    elif command -v vimdiff &> /dev/null; then
        DIFF_TOOL="vimdiff"
    fi
fi

# Validate target directory
if [ ! -d "$TARGET_DIR" ]; then
    echo -e "${RED}Error: Target directory '$TARGET_DIR' does not exist.${NC}" >&2
    exit 1
fi

# Check if .clang-format file exists in the current directory
if [ ! -f ".clang-format" ]; then
    echo -e "${RED}Error: .clang-format file not found in the current directory.${NC}" >&2
    exit 1
fi

# Validate .clang-format file
if $VALIDATE_CONFIG; then
    echo "Validating .clang-format configuration..."
    validation_output=$(clang-format --dump-config 2>&1)
    if [[ $? -ne 0 ]]; then
        echo -e "${RED}Error: Invalid .clang-format configuration:${NC}" >&2
        echo "$validation_output" >&2
        echo -e "\n${YELLOW}Please fix the .clang-format file before proceeding.${NC}" >&2
        echo "Common issues include:"
        echo "  - Duplicate keys (like 'AlignTrailingComments')"
        echo "  - Invalid YAML syntax"
        echo "  - Unsupported clang-format options"
        echo -e "\nUse --no-validate to skip this check at your own risk."
        exit 1
    fi
    echo -e "${GREEN}Configuration validation successful.${NC}"
fi

# We'll create the backup directory only when needed, not upfront

# Add the current backup directory to the exclusion list if we need to create it
EXCLUDE_DIRS+=("$BACKUP_DIR")

# Create temp directory for processing
TEMP_DIR=$(mktemp -d)
trap 'rm -rf "$TEMP_DIR"' EXIT

# Create a file to store the list of files to process
FILES_LIST="$TEMP_DIR/files_list.txt"

# Find all backup directories and add them to exclusions
if [[ -d "." ]]; then
    # Find all directories matching the backup pattern and add to exclusions
    while IFS= read -r backup_dir; do
        if [[ -d "$backup_dir" && "$backup_dir" =~ clang_format_backup_ ]]; then
            # Only add if not already in the list
            if [[ ! " ${EXCLUDE_DIRS[*]} " =~ " ${backup_dir#./} " ]]; then
                EXCLUDE_DIRS+=("${backup_dir#./}")
            fi
        fi
    done < <(find . -maxdepth 1 -type d -name "clang_format_backup_*" 2>/dev/null)
fi

# Build exclude directory pattern for find command
EXCLUDE_PATTERN=""
for dir in "${EXCLUDE_DIRS[@]}"; do
    # Handle both relative and absolute paths for exclusion
    if [[ "$dir" == /* ]]; then
        # Absolute path
        EXCLUDE_PATTERN="$EXCLUDE_PATTERN -path $dir -prune -o"
    else
        # Relative path - could be in current dir or in target dir
        EXCLUDE_PATTERN="$EXCLUDE_PATTERN -path ./$dir -prune -o"
        if [[ "$TARGET_DIR" != "." ]]; then
            EXCLUDE_PATTERN="$EXCLUDE_PATTERN -path $TARGET_DIR/$dir -prune -o"
        fi
    fi
done
EXCLUDE_PATTERN="${EXCLUDE_PATTERN# }"  # Remove leading space

# Build file type pattern for find command
FILE_PATTERN=""
for type in "${FILE_TYPES[@]}"; do
    FILE_PATTERN="$FILE_PATTERN -name \"$type\" -o"
done
FILE_PATTERN="${FILE_PATTERN% -o}"  # Remove trailing -o

# Find files and save them to a temporary file
echo "Finding source files to process in: $TARGET_DIR"
echo "Excluding directories: ${EXCLUDE_DIRS[*]}"
echo "Size threshold: $SIZE_THRESHOLD% (files reduced below this will be flagged)"

# Construct find command safely
FIND_CMD="find $TARGET_DIR"
if [[ -n "$EXCLUDE_PATTERN" ]]; then
    FIND_CMD="$FIND_CMD $EXCLUDE_PATTERN"
fi
FIND_CMD="$FIND_CMD -type f \( $FILE_PATTERN \) -print"

# Execute find command
eval "$FIND_CMD" > "$FILES_LIST" 2>/dev/null

FILE_COUNT=$(wc -l < "$FILES_LIST" | tr -d ' ')
echo "Found $FILE_COUNT files to process."

if [ "$FILE_COUNT" -eq 0 ]; then
    echo -e "${YELLOW}Warning: No files found to process. Check your directory and exclusion settings.${NC}"
    exit 0
fi

# Counter for modified files
MODIFIED_COUNT=0
CHECKED_COUNT=0
ERROR_COUNT=0
REVIEW_FILES=()

# Function to create backup directory if needed
create_backup_dir_if_needed() {
    if ! $BACKUP_CREATED && ! $CHECK_ONLY; then
        mkdir -p "$BACKUP_DIR"
        echo "Created backup directory: $BACKUP_DIR"
        BACKUP_CREATED=true
    fi
}

# Interactive prompt function
prompt_user() {
    local file="$1"
    local temp_file="$2"
    local original_size="$3"
    local formatted_size="$4"

    # Calculate percentage
    local percentage=$((formatted_size * 100 / original_size))
    local reduction=$((100 - percentage))

    echo -e "\n${YELLOW}================ MANUAL REVIEW REQUIRED =================${NC}"
    echo -e "File: ${BOLD}$file${NC}"
    echo "Original size: $original_size bytes, Formatted size: $formatted_size bytes"
    echo "This represents a $reduction% reduction in size."
    echo -e "${YELLOW}Formatting would significantly reduce the file size. This could be valid${NC}"
    echo -e "${YELLOW}reformatting or could indicate potential content loss.${NC}"

    # Create diff for display
    diff -u "$file" "$temp_file" > "$TEMP_DIR/diff_output.txt"

    echo -e "\n${BLUE}Options for handling this file:${NC}"
    echo "  1) View text diff (using 'less')"
    if [[ -n "$DIFF_TOOL" ]]; then
        echo "  2) Open in visual diff tool ($DIFF_TOOL)"
    fi
    echo "  3) Apply formatting changes anyway"
    echo "  4) Skip this file (default)"
    echo "  5) View original file"
    echo "  6) View formatted file"

    local choice
    read -p "Enter choice [1-6]: " choice

    case "$choice" in
        1)
            less "$TEMP_DIR/diff_output.txt"
            # Ask again after viewing
            prompt_user "$file" "$temp_file" "$original_size" "$formatted_size"
            ;;
        2)
            if [[ -n "$DIFF_TOOL" ]]; then
                $DIFF_TOOL "$file" "$temp_file"
                # Ask again after viewing
                prompt_user "$file" "$temp_file" "$original_size" "$formatted_size"
            else
                echo "No visual diff tool available. Please choose another option."
                prompt_user "$file" "$temp_file" "$original_size" "$formatted_size"
            fi
            ;;
        3)
            echo "Applying formatting changes to $file"
            return 0
            ;;
        4|"")
            echo "Skipping $file"
            return 1
            ;;
        5)
            less "$file"
            # Ask again after viewing
            prompt_user "$file" "$temp_file" "$original_size" "$formatted_size"
            ;;
        6)
            less "$temp_file"
            # Ask again after viewing
            prompt_user "$file" "$temp_file" "$original_size" "$formatted_size"
            ;;
        *)
            echo "Invalid choice. Please try again."
            prompt_user "$file" "$temp_file" "$original_size" "$formatted_size"
            ;;
    esac
}

# Process files from the list
while IFS= read -r file; do
    # Get relative path for display and backup
    if [[ "$file" = /* ]]; then
        # File has absolute path
        filepath="${file#$(pwd)/}"
    else
        # File has relative path
        filepath="${file#./}"
    fi

    CHECKED_COUNT=$((CHECKED_COUNT + 1))

    # Save original file permissions
    if [[ "$(uname)" == "Darwin" ]]; then
        original_perms=$(stat -f "%p" "$file" | tail -c 4)  # Extract last 3 digits
    else
        original_perms=$(stat --format "%a" "$file")
    fi

    # Create temporary file for formatted output
    temp_file=$(mktemp "$TEMP_DIR/XXXXXXXX")

    # Run clang-format and save to temp file
    clang-format -style=file "$file" > "$temp_file" 2> "$TEMP_DIR/format_error.log"

    # Check for errors and if temp file is empty
    if [ $? -ne 0 ] || [ ! -s "$temp_file" ]; then
        echo -e "${RED}Error formatting file: $filepath${NC}"
        cat "$TEMP_DIR/format_error.log" >&2
        ERROR_COUNT=$((ERROR_COUNT + 1))
        continue
    fi

    # Safety check: don't replace with empty or significantly smaller file
    original_size=$(wc -c < "$file")
    formatted_size=$(wc -c < "$temp_file")

    # Calculate percentage using integer arithmetic
    percentage=$((formatted_size * 100 / original_size))

    # Alert if the formatted file is much smaller (more than threshold reduction)
    if (( percentage < SIZE_THRESHOLD )); then
        if $INTERACTIVE && ! $CHECK_ONLY; then
            if prompt_user "$file" "$temp_file" "$original_size" "$formatted_size"; then
                # User chose to apply changes despite the warning
                # First, create backup directory if it hasn't been created yet
                create_backup_dir_if_needed

                # Create backup directory structure and backup the file
                backup_file_path="$BACKUP_DIR/$filepath"
                backup_dir=$(dirname "$backup_file_path")
                mkdir -p "$backup_dir"
                cp "$file" "$backup_file_path"

                # Copy the formatted content back to the original file
                cp "$temp_file" "$file"

                # Restore original file permissions
                chmod "$original_perms" "$file"

                echo -e "${GREEN}Applied clang-format to: $filepath${NC} (after manual review)"
                MODIFIED_COUNT=$((MODIFIED_COUNT + 1))
            else
                # User chose to skip this file
                echo -e "${YELLOW}Skipped formatting: $filepath${NC} (after manual review)"
                REVIEW_FILES+=("$filepath")
                continue
            fi
        else
            echo -e "${YELLOW}Warning: Formatting would significantly reduce file size for $filepath${NC}"
            echo "Original: $original_size bytes, Formatted: $formatted_size bytes ($percentage% of original)"
            echo "Skipping this file for safety. Use --interactive to review changes."
            REVIEW_FILES+=("$filepath")
            continue
        fi
    else
        # Compare original and formatted files
        if ! cmp -s "$file" "$temp_file"; then
            MODIFIED_COUNT=$((MODIFIED_COUNT + 1))

            if $CHECK_ONLY; then
                echo "File would be modified: $filepath"
            else
                # First, create backup directory if it hasn't been created yet
                create_backup_dir_if_needed

                # Create backup directory structure and backup the file
                backup_file_path="$BACKUP_DIR/$filepath"
                backup_dir=$(dirname "$backup_file_path")
                mkdir -p "$backup_dir"
                cp "$file" "$backup_file_path"

                # Copy the formatted content back to the original file
                cp "$temp_file" "$file"

                # Restore original file permissions
                chmod "$original_perms" "$file"
                echo -e "${GREEN}Applied clang-format to: $filepath${NC}"
            fi
        fi
    fi

    # Progress indicator for larger projects
    if [ $CHECKED_COUNT -gt 0 ] && [ $((CHECKED_COUNT % 10)) -eq 0 ]; then
        echo -e "${YELLOW}Progress: $CHECKED_COUNT/$FILE_COUNT files processed${NC}"
    fi
done < "$FILES_LIST"

# Summary
echo ""
echo "================================ SUMMARY ================================"
if $CHECK_ONLY; then
    echo -e "Checked ${GREEN}$CHECKED_COUNT${NC} files. ${YELLOW}$MODIFIED_COUNT${NC} file(s) would be modified."
    if [ $ERROR_COUNT -gt 0 ]; then
        echo -e "${RED}$ERROR_COUNT${NC} file(s) had formatting errors."
    fi
    if [ ${#REVIEW_FILES[@]} -gt 0 ]; then
        echo -e "${YELLOW}${#REVIEW_FILES[@]}${NC} file(s) flagged for manual review:"
        for file in "${REVIEW_FILES[@]}"; do
            echo "  - $file"
        done
    fi
    if [ $MODIFIED_COUNT -gt 0 ]; then
        exit 1
    fi
else
    echo -e "Processed ${GREEN}$CHECKED_COUNT${NC} files. Modified ${YELLOW}$MODIFIED_COUNT${NC} file(s)."
    if [ $ERROR_COUNT -gt 0 ]; then
        echo -e "${RED}$ERROR_COUNT${NC} file(s) had formatting errors and were skipped."
    fi
    if [ ${#REVIEW_FILES[@]} -gt 0 ]; then
        echo -e "${YELLOW}${#REVIEW_FILES[@]}${NC} file(s) skipped, requiring manual review:"
        for file in "${REVIEW_FILES[@]}"; do
            echo "  - $file"
        done
        echo ""
        echo "To manually review each file:"
        echo "  1. View original:   cat <filename>"
        echo "  2. View formatted:  clang-format <filename> | less"
        echo "  3. Compare diff:    clang-format <filename> | diff -u <filename> -"
        echo "  4. Apply manually:  clang-format -i <filename>"
    fi

    # Only show backup information if a backup was created
    if $BACKUP_CREATED; then
        echo -e "Backups saved to: ${GREEN}$BACKUP_DIR${NC}"

        # Instructions for restoring from backup if needed
        if [ $MODIFIED_COUNT -gt 0 ]; then
            echo ""
            echo "If you need to restore from backup, run:"
            echo "  for f in \$(find $BACKUP_DIR -type f); do cp \"\$f\" \"\${f#$BACKUP_DIR/}\"; done"
        fi
    else
        echo -e "${GREEN}No files were modified, no backup directory was created.${NC}"
    fi
fi
echo "========================================================================"

# Clean up empty backup directory if it was created but not used
if $BACKUP_CREATED && [ $MODIFIED_COUNT -eq 0 ]; then
    rmdir "$BACKUP_DIR" 2>/dev/null
    echo -e "${YELLOW}Removed empty backup directory: $BACKUP_DIR${NC}"
fi

exit $(( ERROR_COUNT + ${#REVIEW_FILES[@]} ))
