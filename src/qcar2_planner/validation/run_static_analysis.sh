#!/bin/sh

. "$(dirname "$0")/common.sh"

# Disable errexit and nounset to handle errors and missing args manually
set +eu

[ $# -lt 2 ] && {
  log_error "Usage: $0 <clang-tidy-exe> <build-path> [--check] [--passes N] <files...>"
  exit 1
}

ensure_executable_exists "$1" "clang-tidy tool"
clang_tidy_exe="$1"
shift

[ -z "$1" ] && {
  log_error "Build path (containing compile_commands.json) not provided."
  exit 1
}
build_path="$1"
shift

check_only=false
[ $# -gt 0 ] && [ "$1" = "--check" ] && {
  check_only=true
  shift
}

max_passes=3
[ $# -gt 0 ] && [ "$1" = "--passes" ] && {
  shift
  [ -z "$1" ] && {
    log_error "--passes requires a numeric argument."
    exit 1
  }
  max_passes="$1"
  shift
}

ensure_files_exist $#

[ ! -f "${build_path}/compile_commands.json" ] && {
  log_error "compile_commands.json not found in: ${build_path}"
  exit 1
}

project_root="$(cd "$(dirname "$0")"/.. && pwd)"
header_filter="^${project_root}/library/"

# Portable CPU count detection (Linux and macOS)
if command -v nproc > /dev/null 2>&1; then
  num_jobs=$(nproc)
elif command -v sysctl > /dev/null 2>&1; then
  num_jobs=$(sysctl -n hw.ncpu)
else
  num_jobs=1
fi

temp_output=""

cleanup() {
  [ -n "${temp_output}" ] && [ -f "${temp_output}" ] && rm -f "${temp_output}"
}
trap cleanup EXIT

# Sets global last_violations to the violation count from the last pass.
# Avoids return-value truncation (return only handles 0-255).
last_violations=0

run_check() {
  log_info "Running clang-tidy on $# files with ${num_jobs} parallel job(s)..."

  temp_output=$(mktemp "${TMPDIR:-/tmp}/clang-tidy-check.XXXXXX")

  "$clang_tidy_exe" -p "$build_path" -j "$num_jobs" \
    "-header-filter=${header_filter}" \
    "-config-file=${project_root}/.clang-tidy" \
    "$@" >"$temp_output" 2>&1
  exit_code=$?

  [ -s "$temp_output" ] && cat "$temp_output"

  log_header "Static Analysis Summary"
  printf " Total files checked: %s\n" "$#"

  [ "$exit_code" -ne 0 ] && {
    log_warning "Some files have issues that need attention."
    log_footer
    log_error "clang-tidy reported issues."
    exit 1
  }

  log_success "No issues found!"
  log_footer
  exit 0
}

run_pass() {
  pass_num="$1"
  shift

  log_progress "$pass_num" "$max_passes" "Running clang-tidy -fix on $# files..."

  temp_output=$(mktemp "${TMPDIR:-/tmp}/clang-tidy-fix.XXXXXX")

  "$clang_tidy_exe" \
    -p "$build_path" \
    "-header-filter=${header_filter}" \
    "-config-file=${project_root}/.clang-tidy" \
    -fix \
    "$@" >"$temp_output" 2>&1

  [ -s "$temp_output" ] && cat "$temp_output"

  last_violations=$(grep -c "warning:" "$temp_output" 2>/dev/null; true)
}

main() {
  if [ "$check_only" = true ]; then
    log_header "Static Analysis Check"
    log_info "Build path: ${build_path}"
    log_info "Files:"
    for sa_file in "$@"; do
      print_file_list "  ${sa_file}"
    done
    run_check "$@"
  else
    log_header "Lint Fix"
    log_info "Build path: ${build_path}"
    log_info "Max passes: ${max_passes}"
    log_info "Files:"
    for sa_file in "$@"; do
      print_file_list "  ${sa_file}"
    done

    pass=1
    while [ "$pass" -le "$max_passes" ]; do
      run_pass "$pass" "$@"

      [ "$last_violations" -eq 0 ] && {
        log_header "Lint Fix Summary"
        printf " Passes completed : %s\n" "$pass"
        printf " Violations found : 0\n"
        log_success "Converged — no lint violations remaining."
        log_footer
        exit 0
      }

      log_info "Pass ${pass}: ${last_violations} violation(s) remaining."
      pass=$((pass + 1))
    done

    log_header "Lint Fix Summary"
    printf " Passes completed : %s\n" "$max_passes"
    printf " Violations found : %s\n" "$last_violations"
    log_warning "Did not fully converge after ${max_passes} pass(es)."
    log_footer
    log_error "Lint violations remain. Run again or increase --passes."
    exit 1
  fi
}

main "$@"
