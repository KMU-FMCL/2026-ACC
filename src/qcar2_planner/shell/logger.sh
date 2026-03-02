#!/bin/sh
# Shell logging utility with color support

readonly COLOR_RESET="\033[0m"
readonly COLOR_GREEN="\033[32m"
readonly COLOR_YELLOW="\033[33m"
readonly COLOR_RED="\033[31m"
readonly COLOR_BLUE="\033[34m"

_colorize() {
  _colorize_color="$1"
  shift
  printf "%b%s%b\n" "${_colorize_color}" "$*" "${COLOR_RESET}"
}

log_info() {
  printf "%s\n" "[INFO] $*"
}

log_warning() {
  _colorize "$COLOR_YELLOW" "[WARN] $*"
}

log_error() {
  _colorize "$COLOR_RED" "[ERROR] $*" >&2
}

log_success() {
  _colorize "$COLOR_GREEN" "[OK] $*"
}

log_header() {
  printf "\n"
  _colorize "$COLOR_BLUE" "=== $* ==="
}

log_footer() {
  _colorize "$COLOR_BLUE" "================================="
  printf "\n"
}

log_progress() {
  printf "%s\n" "[$1/$2] $3"
}

is_program_installed() {
  command -v "$1" >/dev/null 2>&1
}

ensure_executable_exists() {
  _exe_path="$1"
  _exe_name="${2:-executable}"

  if [ -z "$_exe_path" ]; then
    log_error "$_exe_name path not provided"
    exit 1
  fi

  if [ ! -f "$_exe_path" ]; then
    log_error "$_exe_name not found: $_exe_path"
    exit 1
  fi

  if [ ! -x "$_exe_path" ]; then
    log_error "$_exe_name not executable: $_exe_path"
    exit 1
  fi
}

ensure_files_exist() {
  _file_count="$1"

  if [ "$_file_count" -eq 0 ]; then
    log_error "No files provided"
    exit 1
  fi
}
