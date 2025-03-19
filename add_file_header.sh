#!/usr/bin/env bash
# add_file_header.sh
#
# This script processes .c, .h, .v, .py, .go, .sh, Kconfig, and Kconfig.projbuild files,
# excluding 'build' and 'managed_components' directories.
# It first removes an existing single-line comment at the top if formatted as:
#   - `/* ... */` for C-like files
#   - `# ...` for Python, Go, Shell, and Kconfig files
# Then (if a comment was removed) it removes exactly one subsequent blank line (if present).
# Finally, it inserts a new comment at the top containing the file's relative path.
# For `.sh` scripts, it ensures the shebang (`#!...`) remains the first line.

# Ensure awk is available
if ! command -v awk &> /dev/null; then
    echo "Error: awk is not installed. Please install it and try again." >&2
    exit 1
fi

# Create temp directory for processing
TEMP_DIR=$(mktemp -d)
trap 'rm -rf "$TEMP_DIR"' EXIT

find . \
  -path ./build -prune -o \
  -path ./managed_components -prune -o \
  -type f \( -name "*.c" -o -name "*.h" -o -name "*.v" -o -name "*.py" -o -name "*.go" -o -name "*.sh" -o -name "Kconfig" -o -name "Kconfig.projbuild" \) -print | \
while IFS= read -r file; do

    # Get original file permissions (macOS & Linux compatible)
    if [[ "$(uname)" == "Darwin" ]]; then
        original_perms=$(stat -f "%p" "$file" | tail -c 4)  # Extract last 3 digits
    else
        original_perms=$(stat --format "%a" "$file")  # Linux: Get octal mode
    fi

    # Step 1: Remove an existing single-line header comment at the top,
    # and remove one subsequent blank line if present.
    awk '
        BEGIN { comment_removed = 0 }
        NR == 1 {
            if ($0 ~ /^#!.*/) {
                print; next
            }
            if ($0 ~ /^\/\*.*\*\/$/ || $0 ~ /^# .*/) {
                comment_removed = 1
                next
            }
        }
        NR == 2 && comment_removed == 1 {
            if ($0 ~ /^$/) {
                next
            }
        }
        { print }
    ' "$file" > "$TEMP_DIR/$(basename "$file")"

    # Step 2: Prepare to insert the new header containing the file's relative path.
    filepath="${file#./}"
    new_content="$TEMP_DIR/$(basename "$file")"
    
    # Determine comment style based on file type
    case "$file" in
        *.c|*.h|*.v) new_header="/* $filepath */";;
        *.py|*.go|*.sh|Kconfig|Kconfig.projbuild) new_header="# $filepath";;
        *) continue ;; # Skip unknown file types
    esac

    # Create the final content with the new header
    if ! grep -q "^$new_header" "$new_content"; then
        if [[ "$file" == *.sh ]] && grep -q "^#!" "$new_content"; then
            # If it's a shell script with a shebang, insert the header after the first line
            first_line=$(head -n 1 "$new_content")
            tail -n +2 "$new_content" > "$TEMP_DIR/temp"
            echo "$first_line" > "$TEMP_DIR/final"
            echo "$new_header" >> "$TEMP_DIR/final"
            echo "" >> "$TEMP_DIR/final"
            cat "$TEMP_DIR/temp" >> "$TEMP_DIR/final"
        else
            # Otherwise, insert it at the top
            {
                echo "$new_header"
                echo ""
                cat "$new_content"
            } > "$TEMP_DIR/final"
        fi
        
        # Copy the final content back to the original file
        cp "$TEMP_DIR/final" "$file"
        echo "Added header to $filepath"
    else
        # Header already exists, just copy the processed file back
        cp "$new_content" "$file"
        echo "Header already exists in $filepath"
    fi

    # Restore file permissions
    chmod "$original_perms" "$file"

done
