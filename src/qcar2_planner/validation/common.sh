#!/bin/sh

# Load common logger
. "$(dirname "$0")/../shell/logger.sh"

# Validation-specific helpers
print_file_list() {
  printf "%b%b%b\n" "$COLOR_YELLOW" "$1" "$COLOR_RESET"
}
