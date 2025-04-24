#!/usr/bin/env bash
# check_and_update_kconfig.sh
# Runs kconfcheck on all Kconfig* files (including .projbuild),
# retries up to $max_attempts, and moves any generated .new files
# back to their originals.

set -euo pipefail
IFS=$'\n\t'

max_attempts=10
attempt=0

while (( attempt < max_attempts )); do
  echo "Attempt $((attempt + 1)) of $max_attempts: Running kconfcheck on all Kconfig* files..."

  # Run kconfcheck on every Kconfig* file (includes Kconfig.projbuild, etc.)
  find . -type f -name 'Kconfig*' -exec python -m kconfcheck {} \;
  exit_code=$?

  if [[ $exit_code -eq 0 ]]; then
    echo "kconfcheck completed successfully!"

    # Move any generated .new files (e.g. Kconfig.projbuild.new) back to originals
    find . -type f -name 'Kconfig*.new' -print0 \
      | xargs -0 -I{} sh -c '
          orig="${1%.new}"
          echo "Moving $1 → $orig"
          mv "$1" "$orig"
        ' _ {}

    echo "All Kconfig files checked and updated successfully."
    exit 0
  fi

  echo "kconfcheck failed (exit code $exit_code), retrying..."
  (( attempt++ ))
done

echo "Reached maximum attempts ($max_attempts). kconfcheck did not succeed."
exit 1
