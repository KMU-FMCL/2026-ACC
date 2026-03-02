#!/bin/sh
# Install git hooks
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

. "$SCRIPT_DIR/../shell/logger.sh"

log_info "Installing git hooks..."

git config core.hooksPath .githooks

log_info "Hooks path: .githooks"
log_success "Git hooks installed successfully"
log_footer
