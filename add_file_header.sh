#!/usr/bin/env bash
#
# This script processes all .c, .h, and .v files, excluding 'build' and 'managed_components' directories.
# It first removes an existing single-line comment at the top if it's formatted as `/* ... */`,
# then (if a comment was removed) it removes exactly one subsequent blank line (if present).
# Finally, it inserts a new comment at the top containing the file's relative path.

find . \
  -path ./build -prune -o \
  -path ./managed_components -prune -o \
  -type f \( -name "*.c" -o -name "*.h" -o -name "*.v" \) -print | \
while IFS= read -r file; do

    # Step 1: Remove an existing single-line header comment at the top,
    # and remove one subsequent blank line if present.
    awk '
        BEGIN { comment_removed = 0 }
        NR == 1 {
            if ($0 ~ /^\/\*.*\*\/$/) {
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
    ' "$file" > "$file.tmp" && mv "$file.tmp" "$file"

    # Step 2: Prepare to insert the new header containing the file's relative path.
    filepath="${file#./}"
    escaped_filepath=$(printf '%s\n' "$filepath" | sed 's/[\/&]/\\&/g')

    # Insert the new header only if it doesn't already exist in the file.
    if ! grep -q "^/\* $escaped_filepath \*/" "$file"; then
        # For macOS, use: sed -i '' "script" file
        sed -i '' "1s;^;/* $filepath */\n\n;" "$file"
        echo "Added header to $filepath"
    else
        echo "Header already exists in $filepath"
    fi

done

