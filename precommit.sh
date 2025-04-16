#!/usr/bin/env bash
# precommit.sh
#
# A comprehensive pre-commit script that runs various code quality checks
# before allowing a commit to proceed. It integrates existing tools like
# clang-format, trailing whitespace removal, file header verification,
# and Kconfig validation.
#
# Usage: Add to your Git hooks by running:
#   cp precommit.sh .git/hooks/pre-commit
#   chmod +x .git/hooks/pre-commit

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Print header function
print_header() {
    echo -e "\n${BLUE}${BOLD}======== $1 ========${NC}"
}

# Get list of staged files (only process files that are being committed)
get_staged_files() {
    local file_pattern="$1"
    git diff --cached --name-only --diff-filter=ACM | grep -E "$file_pattern" || true
}

# Function to check if any command exists
check_command() {
    if ! command -v "$1" &> /dev/null; then
        echo -e "${YELLOW}Warning: $1 is not installed. Skipping $2 check.${NC}"
        return 1
    fi
    return 0
}

# Initialize variables to track overall success
ALL_CHECKS_PASSED=true
REPORT=""

# Create temp directory for processing
TEMP_DIR=$(mktemp -d)
trap 'rm -rf "$TEMP_DIR"' EXIT

# Function to append to the report
append_report() {
    REPORT="${REPORT}\n$1"
}

# -----------------------------------------------------------------------------
# 1. Check for trailing whitespace
# -----------------------------------------------------------------------------
print_header "Checking for trailing whitespace"

if [ -f "./remove_trailing_whitespace.sh" ]; then
    echo "Using existing remove_trailing_whitespace.sh script"
    if ! ./remove_trailing_whitespace.sh --check; then
        echo -e "${YELLOW}Found files with trailing whitespace.${NC}"
        echo -e "${YELLOW}Run './remove_trailing_whitespace.sh --fix' to fix them.${NC}"
        ALL_CHECKS_PASSED=false
        append_report "${RED}✗ Trailing whitespace check failed${NC}"
    else
        echo -e "${GREEN}No trailing whitespace found in staged files.${NC}"
        append_report "${GREEN}✓ Trailing whitespace check passed${NC}"
    fi
else
    echo "Custom whitespace check (remove_trailing_whitespace.sh not found)"
    WHITESPACE_FILES=$(get_staged_files '\.(c|h|py|sh|v|go|md|txt)$|Kconfig')
    if [ -n "$WHITESPACE_FILES" ]; then
        WHITESPACE_FOUND=false
        for file in $WHITESPACE_FILES; do
            if [ -f "$file" ] && grep -q '[[:space:]]$' "$file"; then
                echo -e "${YELLOW}Trailing whitespace found in: $file${NC}"
                WHITESPACE_FOUND=true
            fi
        done

        if $WHITESPACE_FOUND; then
            ALL_CHECKS_PASSED=false
            append_report "${RED}✗ Trailing whitespace check failed${NC}"
        else
            echo -e "${GREEN}No trailing whitespace found in staged files.${NC}"
            append_report "${GREEN}✓ Trailing whitespace check passed${NC}"
        fi
    else
        echo "No relevant files found for whitespace check."
        append_report "${GREEN}✓ Trailing whitespace check skipped (no relevant files)${NC}"
    fi
fi

# -----------------------------------------------------------------------------
# 2. Check code formatting with clang-format
# -----------------------------------------------------------------------------
print_header "Checking code formatting with clang-format"

if check_command "clang-format" "code formatting"; then
    if [ -f "./.clang-format" ]; then
        if [ -f "./apply_clang_format.sh" ]; then
            echo "Using existing apply_clang_format.sh script"
            if ! ./apply_clang_format.sh --check; then
                echo -e "${YELLOW}Found files with formatting issues.${NC}"
                echo -e "${YELLOW}Run './apply_clang_format.sh --fix' to fix them.${NC}"
                ALL_CHECKS_PASSED=false
                append_report "${RED}✗ Code formatting check failed${NC}"
            else
                echo -e "${GREEN}All files are properly formatted.${NC}"
                append_report "${GREEN}✓ Code formatting check passed${NC}"
            fi
        else
            echo "Custom clang-format check (apply_clang_format.sh not found)"
            CLANG_FILES=$(get_staged_files '\.(c|cpp|h|hpp)$')
            if [ -n "$CLANG_FILES" ]; then
                FORMATTING_ISSUES=false
                for file in $CLANG_FILES; do
                    if [ -f "$file" ]; then
                        diff -u "$file" <(clang-format -style=file "$file") > /dev/null || {
                            echo -e "${YELLOW}Formatting issues found in: $file${NC}"
                            FORMATTING_ISSUES=true
                        }
                    fi
                done

                if $FORMATTING_ISSUES; then
                    ALL_CHECKS_PASSED=false
                    append_report "${RED}✗ Code formatting check failed${NC}"
                else
                    echo -e "${GREEN}All files are properly formatted.${NC}"
                    append_report "${GREEN}✓ Code formatting check passed${NC}"
                fi
            else
                echo "No relevant files found for clang-format check."
                append_report "${GREEN}✓ Code formatting check skipped (no relevant files)${NC}"
            fi
        fi
    else
        echo -e "${YELLOW}No .clang-format file found. Skipping code formatting check.${NC}"
        append_report "${YELLOW}⚠ Code formatting check skipped (no .clang-format file)${NC}"
    fi
else
    append_report "${YELLOW}⚠ Code formatting check skipped (clang-format not installed)${NC}"
fi

# -----------------------------------------------------------------------------
# 3. Check file headers
# -----------------------------------------------------------------------------
print_header "Checking file headers"

if [ -f "./add_file_header.sh" ]; then
    echo "Using existing add_file_header.sh for header check"

    # Modified approach: Check headers manually without modifying the script
    HEADER_CHECK_FAILED=false
    HEADER_FILES=$(get_staged_files '\.(c|h|v|py|go|sh)$|CMakeLists\.txt|Kconfig')

    if [ -n "$HEADER_FILES" ]; then
        for file in $HEADER_FILES; do
            if [ -f "$file" ]; then
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

                # Determine the current header line
                if [[ "$file" == *.sh ]]; then
                    # For shell scripts, if a shebang exists, check the second line;
                    # otherwise, use the first line.
                    if head -n 1 "$file" | grep -q "^#!"; then
                        header_line=$(head -n 2 "$file" | tail -n 1)
                    else
                        header_line=$(head -n 1 "$file")
                    fi
                else
                    header_line=$(head -n 1 "$file")
                fi

                # If the header doesn't match the expected header, flag it
                if [[ "$header_line" != "$expected_header" ]]; then
                    echo -e "${YELLOW}Missing or incorrect header in: $filepath${NC}"
                    echo -e "${YELLOW}Expected: $expected_header${NC}"
                    echo -e "${YELLOW}Found:    $header_line${NC}"
                    HEADER_CHECK_FAILED=true
                fi
            fi
        done

        if $HEADER_CHECK_FAILED; then
            echo -e "${YELLOW}Run './add_file_header.sh' to fix headers.${NC}"
            ALL_CHECKS_PASSED=false
            append_report "${RED}✗ File header check failed${NC}"
        else
            echo -e "${GREEN}All files have correct headers.${NC}"
            append_report "${GREEN}✓ File header check passed${NC}"
        fi
    else
        echo "No relevant files found for header check."
        append_report "${GREEN}✓ File header check skipped (no relevant files)${NC}"
    fi
else
    echo -e "${YELLOW}add_file_header.sh not found. Skipping file header check.${NC}"
    append_report "${YELLOW}⚠ File header check skipped (add_file_header.sh not found)${NC}"
fi

# -----------------------------------------------------------------------------
# 4. Check for consecutive blank lines
# -----------------------------------------------------------------------------
print_header "Checking for consecutive blank lines"

if [ -f "./find_consecutive_blank_lines.sh" ]; then
    echo "Using existing find_consecutive_blank_lines.sh script"
    BLANK_LINES_OUTPUT=$("./find_consecutive_blank_lines.sh")
    if [ -n "$BLANK_LINES_OUTPUT" ]; then
        echo -e "${YELLOW}Found files with consecutive blank lines:${NC}"
        echo "$BLANK_LINES_OUTPUT"
        ALL_CHECKS_PASSED=false
        append_report "${RED}✗ Consecutive blank lines check failed${NC}"
    else
        echo -e "${GREEN}No consecutive blank lines found.${NC}"
        append_report "${GREEN}✓ Consecutive blank lines check passed${NC}"
    fi
else
    echo "Custom blank lines check (find_consecutive_blank_lines.sh not found)"
    # Check for consecutive blank lines in relevant files
    BLANK_LINE_FILES=$(get_staged_files '\.(c|h)$')
    if [ -n "$BLANK_LINE_FILES" ]; then
        BLANK_LINES_FOUND=false
        for file in $BLANK_LINE_FILES; do
            if [ -f "$file" ]; then
                CONSECUTIVE_BLANKS=$(awk '
                    BEGIN { blank_count = 0 }
                    NF == 0 {
                        blank_count++
                        if (blank_count == 2)
                            print FILENAME ":" FNR ": [second blank line]"
                        next
                    }
                    { blank_count = 0 }
                ' "$file")

                if [ -n "$CONSECUTIVE_BLANKS" ]; then
                    echo -e "${YELLOW}Consecutive blank lines found in: $file${NC}"
                    echo "$CONSECUTIVE_BLANKS"
                    BLANK_LINES_FOUND=true
                fi
            fi
        done

        if $BLANK_LINES_FOUND; then
            ALL_CHECKS_PASSED=false
            append_report "${RED}✗ Consecutive blank lines check failed${NC}"
        else
            echo -e "${GREEN}No consecutive blank lines found in staged files.${NC}"
            append_report "${GREEN}✓ Consecutive blank lines check passed${NC}"
        fi
    else
        echo "No relevant files found for blank lines check."
        append_report "${GREEN}✓ Consecutive blank lines check skipped (no relevant files)${NC}"
    fi
fi

# -----------------------------------------------------------------------------
# 5. Kconfig check
# -----------------------------------------------------------------------------
print_header "Checking Kconfig files"

KCONFIG_FILES=$(get_staged_files 'Kconfig')
if [ -n "$KCONFIG_FILES" ]; then
    if check_command "python" "Kconfig validation" && python -c "import kconfcheck" &>/dev/null; then
        echo "Running Kconfig validation"
        KCONFIG_ISSUES=false

        for file in $KCONFIG_FILES; do
            if [ -f "$file" ]; then
                echo "Checking $file..."
                # Run kconfcheck with the --check syntax flag
                if ! python -m kconfcheck --check syntax "$file" > "$TEMP_DIR/kconfcheck_out.log" 2>&1; then
                    echo -e "${YELLOW}Kconfig issues found in: $file${NC}"
                    cat "$TEMP_DIR/kconfcheck_out.log"
                    KCONFIG_ISSUES=true
                else
                    # Check if any syntax issues were reported in the output
                    if grep -i "error\|warning\|issue" "$TEMP_DIR/kconfcheck_out.log" > /dev/null; then
                        echo -e "${YELLOW}Kconfig issues found in: $file${NC}"
                        cat "$TEMP_DIR/kconfcheck_out.log"
                        KCONFIG_ISSUES=true
                    fi
                fi
            fi
        done

        if $KCONFIG_ISSUES; then
            ALL_CHECKS_PASSED=false
            append_report "${RED}✗ Kconfig validation failed${NC}"
        else
            echo -e "${GREEN}All Kconfig files are valid.${NC}"
            append_report "${GREEN}✓ Kconfig validation passed${NC}"
        fi
    else
        echo -e "${YELLOW}kconfcheck module not found. Skipping Kconfig validation.${NC}"
        append_report "${YELLOW}⚠ Kconfig validation skipped (kconfcheck not installed)${NC}"
    fi
else
    echo "No Kconfig files found for validation."
    append_report "${GREEN}✓ Kconfig validation skipped (no Kconfig files)${NC}"
fi

# -----------------------------------------------------------------------------
# Final Report
# -----------------------------------------------------------------------------
print_header "Pre-commit Check Summary"
echo -e "$REPORT"

if ! $ALL_CHECKS_PASSED; then
    echo -e "\n${RED}${BOLD}Pre-commit checks failed!${NC}"
    echo -e "${YELLOW}Fix the issues above and try committing again.${NC}"
    echo -e "${YELLOW}Or use 'git commit --no-verify' to bypass these checks (not recommended).${NC}"
    exit 1
else
    echo -e "\n${GREEN}${BOLD}All pre-commit checks passed!${NC}"
    exit 0
fi
