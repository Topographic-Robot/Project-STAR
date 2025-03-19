#!/usr/bin/env bash
# check_and_update_kconfig.sh

max_attempts=10
attempt=0

while [ $attempt -lt $max_attempts ]; do
  echo "Attempt $((attempt + 1)) of $max_attempts: Running kconfcheck on all Kconfig files..."

  # Run kconfcheck on all Kconfig files
  find . -name "Kconfig*" -exec python -m kconfcheck {} \;

  # Capture exit code
  exit_code=$?

  if [ $exit_code -eq 0 ]; then
    echo "kconfcheck completed successfully!"
    
    # Move Kconfig.new files to replace originals
    find . -name "Kconfig.new" -exec sh -c 'echo "Moving $0 to ${0%.new}"; mv "$0" "${0%.new}"' {} \;
    
    if [ $? -eq 0 ]; then
      echo "All Kconfig files checked and updated successfully."
      exit 0
    else
      echo "Error moving Kconfig.new files."
      exit 1
    fi
  fi

  echo "kconfcheck failed (exit code $exit_code), retrying..."
  attempt=$((attempt + 1))
done

echo "Reached maximum attempts ($max_attempts). kconfcheck did not succeed."
exit 1
