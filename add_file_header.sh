#!/usr/bin/env bash
# add_file_header.sh


# add_file_header.sh
#
# This script processes .c, .h, .v, .py, .go, .sh, CMakeLists.txt, Kconfig, and Kconfig.projbuild files,
# excluding 'build', 'managed_components', 'clang_format_backup_*', and 'diff_backup_*' directories.
# It first checks whether the file's header (the first non-shebang line for .sh files)
# matches the expected header (which is based on the file's relative path).
# If the header is already present, no changes are made.
# Otherwise, it removes an existing single-line header comment at the top (if found)
# and then inserts a new header containing the file's relative path.
# For .sh scripts, the shebang line remains the first line.
# For .py files, it ensures #!/usr/bin/env python3 is at the top.

# Ensure awk is available
if ! command -v awk &> /dev/null; then
    echo "Error: awk is not installed. Please install it and try again." >&2
    exit 1
fi

# Create temp directory for processing
TEMP_DIR=$(mktemp -d)
trap 'rm -rf "$TEMP_DIR"' EXIT

# Modified find command to also exclude clang_format_backup_* and diff_backup_* directories
find . \
  -path ./build -prune -o \
  -path ./managed_components -prune -o \
  -path "./clang_format_backup_*" -prune -o \
  -path "./diff_backup_*" -prune -o \
  -type f \( -name "*.c" -o -name "*.h" -o -name "*.v" -o -name "*.py" -o -name "*.go" -o -name "*.sh" -o -name "CMakeLists.txt" -o -name "Kconfig" -o -name "Kconfig.projbuild" \) -print | \
while IFS= read -r file; do

    # Get the file's relative path (remove leading "./")
    filepath="${file#./}"

    # Determine expected header based on file type
    filename=$(basename "$file")
    if [[ "$file" == *.c || "$file" == *.h || "$file" == *.v || "$file" == *.go ]]; then
        expected_header="/* $filepath */"
    elif [[ "$file" == *.py || "$file" == *.sh || "$filename" == "Kconfig" || "$filename" == "Kconfig.projbuild" || "$filename" == "CMakeLists.txt" ]]; then
        expected_header="# $filepath"
    else
        continue  # Skip unknown file types
    fi

    # Check for Python files to add shebang
    if [[ "$file" == *.py ]]; then
        python_shebang="#!/usr/bin/env python3"
        first_line=$(head -n 1 "$file")

        # If there's no shebang or wrong shebang, we'll need to add/replace it
        needs_python_shebang=0
        if [[ "$first_line" != "$python_shebang" ]]; then
            needs_python_shebang=1
        fi
    fi

    # Determine the current header line
    if [[ "$file" == *.sh ]]; then
        # For shell scripts, if a shebang exists, check the second line;
        # otherwise, use the first line.
        if head -n 1 "$file" | grep -q "^#!"; then
            header_line=$(head -n 2 "$file" | tail -n 1)
        else
            header_line=$(head -n 1 "$file")
        fi
    elif [[ "$file" == *.py ]]; then
        # For Python scripts, if a shebang exists, check the second line;
        # otherwise, use the first line.
        if head -n 1 "$file" | grep -q "^#!"; then
            header_line=$(head -n 2 "$file" | tail -n 1)
        else
            header_line=$(head -n 1 "$file")
        fi
    else
        header_line=$(head -n 1 "$file")
    fi

    # If the header already matches the expected header and Python files have correct shebang, skip processing
    if [[ "$header_line" == "$expected_header" ]] && ( [[ "$file" != *.py ]] || [[ "$needs_python_shebang" == 0 ]] ); then
        echo "Header already exists in $filepath"
        continue
    fi

    # Save original file permissions (compatible with macOS & Linux)
    if [[ "$(uname)" == "Darwin" ]]; then
        original_perms=$(stat -f "%p" "$file" | tail -c 4)  # Extract last 3 digits
    else
        original_perms=$(stat --format "%a" "$file")
    fi

    # Create a unique temporary file for processing this file
    temp_file=$(mktemp "$TEMP_DIR/XXXXXXXX")

    # Step 1: Remove an existing single-line header comment at the top
    # (if formatted as either a C-style comment or a hash-comment),
    # and remove one subsequent blank line (if present).
    awk -v shebang="^#!.*" -v comment_re="^(\/\*.*\*\/|# .*)" '
        BEGIN { comment_removed = 0 }
        NR == 1 {
            if ($0 ~ shebang) {
                print; next
            }
            if ($0 ~ comment_re) {
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
    ' "$file" > "$temp_file"

    # Step 2: Insert the new header containing the file's relative path.
    # Use a different temporary file for final content.
    final_temp=$(mktemp "$TEMP_DIR/XXXXXXXX")

    if [[ "$file" == *.py ]]; then
        # Python file handling with proper shebang
        if [[ "$needs_python_shebang" == 1 ]]; then
            # First check if there's already a shebang (but wrong one) to replace
            if head -n 1 "$temp_file" | grep -q "^#!"; then
                # Replace existing shebang
                tail -n +2 "$temp_file" > "$TEMP_DIR/temp_content"
                {
                    echo "$python_shebang"
                    echo "$expected_header"
                    echo ""
                    cat "$TEMP_DIR/temp_content"
                } > "$final_temp"
            else
                # No shebang, add one
                {
                    echo "$python_shebang"
                    echo "$expected_header"
                    echo ""
                    cat "$temp_file"
                } > "$final_temp"
            fi
        else
            # Correct shebang exists, just update the header if needed
            first_line=$(head -n 1 "$temp_file")
            tail -n +2 "$temp_file" > "$TEMP_DIR/temp_content"
            {
                echo "$first_line"
                echo "$expected_header"
                echo ""
                cat "$TEMP_DIR/temp_content"
            } > "$final_temp"
        fi
    elif [[ "$file" == *.sh ]] && grep -q "^#!" "$temp_file"; then
        # If it's a shell script with a shebang, preserve the first line.
        first_line=$(head -n 1 "$temp_file")
        tail -n +2 "$temp_file" > "$TEMP_DIR/temp_content"
        {
            echo "$first_line"
            echo "$expected_header"
            echo ""
            cat "$TEMP_DIR/temp_content"
        } > "$final_temp"
    else
        {
            echo "$expected_header"
            echo ""
            cat "$temp_file"
        } > "$final_temp"
    fi

    # Copy the final content back to the original file
    cp "$final_temp" "$file"
    echo "Added header to $filepath"

    # Restore original file permissions
    chmod "$original_perms" "$file"
done
