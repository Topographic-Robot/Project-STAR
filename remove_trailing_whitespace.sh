#!/usr/bin/env bash
# remove_trailing_whitespace.sh
#
# This script finds and removes trailing whitespace from specified file types.
# It processes files in the current directory and subdirectories, excluding
# certain directories like 'build' and 'managed_components'.
# The script can operate in two modes:
#   1. Check-only: Reports files with trailing whitespace without modifying them
#   2. Fix: Removes trailing whitespace from affected files

# Default settings
CHECK_ONLY=false
FILE_TYPES=("*.c" "*.h" "*.py" "*.sh" "*.v" "*.go" "Kconfig" "Kconfig.projbuild" "*.md" "*.txt")
EXCLUDE_DIRS=("build" "managed_components")

# Ensure required commands are available
if ! command -v sed &> /dev/null; then
  echo "Error: sed is not installed. Please install it and try again." >&2
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
    --help)
      echo "Usage: $0 [OPTIONS]"
      echo "Remove trailing whitespace from source files."
      echo ""
      echo "Options:"
      echo "  --check    Only check and report files with trailing whitespace (default: false)"
      echo "  --fix      Fix files by removing trailing whitespace (default action)"
      echo "  --help     Display this help message and exit"
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Use --help for usage information" >&2
      exit 1
      ;;
  esac
done

# Build exclude directory pattern for find command
EXCLUDE_PATTERN=""
for dir in "${EXCLUDE_DIRS[@]}"; do
  EXCLUDE_PATTERN="$EXCLUDE_PATTERN -path ./$dir -prune -o"
done
EXCLUDE_PATTERN="${EXCLUDE_PATTERN# }"  # Remove leading space

# Build file type pattern for find command
FILE_PATTERN=""
for type in "${FILE_TYPES[@]}"; do
  FILE_PATTERN="$FILE_PATTERN -name \"$type\" -o"
done
FILE_PATTERN="${FILE_PATTERN% -o}"  # Remove trailing -o

# Create temporary directory for processing
TEMP_DIR=$(mktemp -d)
trap 'rm -rf "$TEMP_DIR"' EXIT

# Determine sed command based on OS (macOS requires different syntax)
if [[ "$(uname)" == "Darwin" ]]; then
  SED_CMD="sed -i '' -E 's/[[:space:]]+$//g'"
else
  SED_CMD="sed -i -E 's/[[:space:]]+$//g'"
fi

# Find files with trailing whitespace
echo "Searching for files with trailing whitespace..."
FOUND_FILES=()

# Use eval to properly handle the complex find command with multiple patterns
eval "find . $EXCLUDE_PATTERN -type f \( $FILE_PATTERN \) -print" | \
while IFS= read -r file; do
  # Check if file has trailing whitespace
  if grep -q '[[:space:]]$' "$file"; then
    FOUND_FILES+=("$file")
    filepath="${file#./}"  # Remove leading "./"

    if $CHECK_ONLY; then
      echo "Trailing whitespace found in: $filepath"
    else
      # Save original file permissions
      if [[ "$(uname)" == "Darwin" ]]; then
        original_perms=$(stat -f "%p" "$file" | tail -c 4)
      else
        original_perms=$(stat --format "%a" "$file")
      fi

      # Create temporary file
      temp_file=$(mktemp "$TEMP_DIR/XXXXXXXX")

      # Remove trailing whitespace
      eval "$SED_CMD '$file'" 2>/dev/null || {
        # If the inline sed fails, try with temp file approach
        sed -E 's/[[:space:]]+$//g' "$file" > "$temp_file"
        cp "$temp_file" "$file"
      }

      # Restore original permissions
      chmod "$original_perms" "$file"

      echo "Removed trailing whitespace from: $filepath"
    fi
  fi
done

# Summary
if $CHECK_ONLY; then
  if [ ${#FOUND_FILES[@]} -eq 0 ]; then
    echo "No files with trailing whitespace found."
    exit 0
  else
    echo "${#FOUND_FILES[@]} file(s) contain trailing whitespace."
    exit 1
  fi
else
  if [ ${#FOUND_FILES[@]} -eq 0 ]; then
    echo "No files with trailing whitespace found."
  else
    echo "Successfully processed ${#FOUND_FILES[@]} file(s)."
  fi
  exit 0
fi
