# =============================================================================
# Code Validation Tools Configuration
# =============================================================================

# =============================================================================
# Ament Linters (registered as CTest tests)
# =============================================================================
include("${PROJECT_ROOT}/ament/validation/validations.cmake")

# copyright: license header validation
ament_copyright()

# clang-format: formatting check using project config
ament_clang_format(
  CONFIG_FILE "${PROJECT_ROOT}/.clang-format"
  "${PROJECT_LIBRARY_PATH}"
  "${PROJECT_TEST_PATH}"
)

# CMake style check
ament_lint_cmake()

# XML validation (explicit path required when called from subdirectory)
ament_xmllint("${PROJECT_ROOT}/package.xml")

# clang-tidy: static analysis using project config
ament_clang_tidy(
  "${CMAKE_BINARY_DIR}"
  CONFIG_FILE "${PROJECT_ROOT}/.clang-tidy")

# cppcheck: additional static analysis
ament_cppcheck(
  LANGUAGE "c++"
  "${PROJECT_LIBRARY_PATH}"
  "${PROJECT_TEST_PATH}"
)

# =============================================================================
# Reconfigure Target (ensures file list is up-to-date)
# =============================================================================
add_custom_target(reconfigure
  COMMAND ${CMAKE_COMMAND} ${CMAKE_SOURCE_DIR} -B ${CMAKE_BINARY_DIR}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Reconfiguring to update file lists..."
)

# =============================================================================
# Find Validation Tools
# =============================================================================
find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
# Parallel wrapper for lint_check (uses -j for multi-file parallel execution)
find_program(CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy-18 run-clang-tidy)
# Direct binary for lint_fix (bypasses export-fixes YAML path; required for naming fix-it hints)
find_program(CLANG_TIDY_DIRECT_EXECUTABLE NAMES clang-tidy-18 clang-tidy)
find_program(GCOVR_EXECUTABLE NAMES gcovr)

if(ENABLE_COVERAGE AND NOT GCOVR_EXECUTABLE)
  message(WARNING "ENABLE_COVERAGE=ON but gcovr was not found; coverage targets will not be created")
endif()

# =============================================================================
# Coverage Targets (only available when ENABLE_COVERAGE=ON)
# =============================================================================
if(GCOVR_EXECUTABLE AND ENABLE_COVERAGE)
  if(NOT "${COVERAGE_TARGET_PERCENT}" MATCHES "^[0-9]+$")
    message(FATAL_ERROR
      "COVERAGE_TARGET_PERCENT must be a non-negative integer, "
      "got: '${COVERAGE_TARGET_PERCENT}'"
    )
  endif()

  set(gcovr_base_args
    --root "${PROJECT_ROOT}"
    --exclude "${PROJECT_TEST_PATH}"
    --exclude "${CMAKE_BINARY_DIR}/_deps"
    "${CMAKE_BINARY_DIR}"
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    list(APPEND gcovr_base_args --gcov-executable "llvm-cov gcov")
  endif()

  add_custom_target(coverage
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/coverage"
    COMMAND ${GCOVR_EXECUTABLE}
      ${gcovr_base_args}
      --html-details "${CMAKE_BINARY_DIR}/coverage/index.html"
      --xml-pretty "${CMAKE_BINARY_DIR}/coverage/coverage.xml"
      --txt "${CMAKE_BINARY_DIR}/coverage/coverage.txt"
      --print-summary
      --sort-percentage
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running tests and generating coverage report..."
    VERBATIM
  )

  # Note: ctest re-run is intentional; each target is independently invocable.
  add_custom_target(coverage_check
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    COMMAND ${GCOVR_EXECUTABLE}
      ${gcovr_base_args}
      --print-summary
      --fail-under-line "${COVERAGE_TARGET_PERCENT}"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Running tests and checking coverage threshold (${COVERAGE_TARGET_PERCENT}%)..."
    VERBATIM
  )

  # Note: ctest re-run is intentional; each target is independently invocable.
  add_custom_target(coverage_quick
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    COMMAND ${GCOVR_EXECUTABLE}
      ${gcovr_base_args}
      --print-summary
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Quick coverage summary (${COVERAGE_TARGET_PERCENT}% target)..."
    VERBATIM
  )

  # Note: ctest re-run is intentional; each target is independently invocable.
  add_custom_target(verify_coverage
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    COMMAND ${GCOVR_EXECUTABLE}
      ${gcovr_base_args}
      --print-summary
      --fail-under-line "${COVERAGE_TARGET_PERCENT}"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Verifying coverage >= ${COVERAGE_TARGET_PERCENT}%..."
    VERBATIM
  )
endif()

# =============================================================================
# Collect Source Files
# =============================================================================
if(CI_MODE)
  # CI: All files
  file(GLOB_RECURSE format_files CONFIGURE_DEPENDS
    ${PROJECT_LIBRARY_PATH}/*.cpp
    ${PROJECT_LIBRARY_PATH}/*.hpp
    ${PROJECT_LIBRARY_PATH}/*.h
    ${PROJECT_TEST_PATH}/*.cpp
    ${PROJECT_TEST_PATH}/*.hpp
    ${PROJECT_TEST_PATH}/*.h
  )
  file(GLOB_RECURSE lint_files CONFIGURE_DEPENDS
    ${PROJECT_LIBRARY_PATH}/*.cpp
    ${PROJECT_TEST_PATH}/*.cpp
  )
else()
  # Dev: Changed files only (staged + unstaged + untracked)
  execute_process(
    COMMAND git diff --name-only --diff-filter=ACMR HEAD -- "*.cpp" "*.hpp" "*.h"
    WORKING_DIRECTORY ${PROJECT_ROOT}
    OUTPUT_VARIABLE changed_format
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND git diff --name-only --cached --diff-filter=ACMR -- "*.cpp" "*.hpp" "*.h"
    WORKING_DIRECTORY ${PROJECT_ROOT}
    OUTPUT_VARIABLE staged_format
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND git ls-files --others --exclude-standard -- "*.cpp" "*.hpp" "*.h"
    WORKING_DIRECTORY ${PROJECT_ROOT}
    OUTPUT_VARIABLE untracked_format
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  set(format_files_str "${changed_format}\n${staged_format}\n${untracked_format}")
  string(REPLACE "\n" ";" format_files "${format_files_str}")
  list(REMOVE_DUPLICATES format_files)
  list(FILTER format_files EXCLUDE REGEX "^$")
  list(TRANSFORM format_files PREPEND "${PROJECT_ROOT}/")

  execute_process(
    COMMAND git diff --name-only --diff-filter=ACMR HEAD -- "library/*.cpp" "test/*.cpp"
    WORKING_DIRECTORY ${PROJECT_ROOT}
    OUTPUT_VARIABLE changed_lint
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND git diff --name-only --cached --diff-filter=ACMR -- "library/*.cpp" "test/*.cpp"
    WORKING_DIRECTORY ${PROJECT_ROOT}
    OUTPUT_VARIABLE staged_lint
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND git ls-files --others --exclude-standard -- "library/*.cpp" "test/*.cpp"
    WORKING_DIRECTORY ${PROJECT_ROOT}
    OUTPUT_VARIABLE untracked_lint
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  set(lint_files_str "${changed_lint}\n${staged_lint}\n${untracked_lint}")
  string(REPLACE "\n" ";" lint_files "${lint_files_str}")
  list(REMOVE_DUPLICATES lint_files)
  list(FILTER lint_files EXCLUDE REGEX "^$")
  list(TRANSFORM lint_files PREPEND "${PROJECT_ROOT}/")
endif()

# =============================================================================
# Validation Targets
# =============================================================================
list(LENGTH format_files format_count)
list(LENGTH lint_files lint_count)

if(CI_MODE)
  set(validation_mode "all")
else()
  set(validation_mode "changed")
endif()

if(CLANG_FORMAT_EXECUTABLE)
  if(format_count GREATER 0)
    add_custom_target(format_check
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --dry-run --Werror ${format_files}
      WORKING_DIRECTORY ${PROJECT_ROOT}
      COMMENT "Checking ${validation_mode} files formatting (${format_count} files)..."
      VERBATIM
    )
    add_custom_target(format_fix
      COMMAND ${CLANG_FORMAT_EXECUTABLE} -i ${format_files}
      WORKING_DIRECTORY ${PROJECT_ROOT}
      COMMENT "Fixing ${validation_mode} files formatting (${format_count} files)..."
      VERBATIM
    )
  else()
    add_custom_target(format_check
      COMMAND ${CMAKE_COMMAND} -E echo "No files to format check"
      COMMENT "No files to format check"
    )
    add_custom_target(format_fix
      COMMAND ${CMAKE_COMMAND} -E echo "No files to format"
      COMMENT "No files to format"
    )
  endif()
endif()

if(CLANG_TIDY_EXECUTABLE)
  if(lint_count GREATER 0)
    add_custom_target(lint_check
      COMMAND ${PROJECT_ROOT}/validation/run_static_analysis.sh
        ${CLANG_TIDY_EXECUTABLE} ${CMAKE_BINARY_DIR} --check ${lint_files}
      WORKING_DIRECTORY ${PROJECT_ROOT}
      COMMENT "Checking ${validation_mode} files lint (${lint_count} files)..."
      VERBATIM
    )
  else()
    add_custom_target(lint_check
      COMMAND ${CMAKE_COMMAND} -E echo "No files to lint check"
      COMMENT "No files to lint check"
    )
  endif()
endif()

if(CLANG_TIDY_DIRECT_EXECUTABLE)
  if(lint_count GREATER 0)
    add_custom_target(lint_fix
      COMMAND ${PROJECT_ROOT}/validation/run_static_analysis.sh
        ${CLANG_TIDY_DIRECT_EXECUTABLE} ${CMAKE_BINARY_DIR} ${lint_files}
      WORKING_DIRECTORY ${PROJECT_ROOT}
      COMMENT "Fixing ${validation_mode} files lint (${lint_count} files)..."
      VERBATIM
    )
  else()
    add_custom_target(lint_fix
      COMMAND ${CMAKE_COMMAND} -E echo "No files to lint"
      COMMENT "No files to lint"
    )
  endif()
endif()

# =============================================================================
# Combined Validation Target
# =============================================================================
if(CLANG_FORMAT_EXECUTABLE OR CLANG_TIDY_EXECUTABLE)
  add_custom_target(validate)
  if(CLANG_FORMAT_EXECUTABLE)
    add_dependencies(validate format_fix)
  endif()
  if(CLANG_TIDY_EXECUTABLE)
    add_dependencies(validate lint_check)
  endif()
endif()

# =============================================================================
# Enforce Validation (format before lint)
# =============================================================================
if(CLANG_FORMAT_EXECUTABLE AND CLANG_TIDY_EXECUTABLE)
  add_dependencies(lint_check format_fix)
endif()

# =============================================================================
# Auto-reconfigure before validation (ensures file list is current)
# =============================================================================
if(CLANG_FORMAT_EXECUTABLE)
  add_dependencies(format_check reconfigure)
  add_dependencies(format_fix reconfigure)
endif()
if(CLANG_TIDY_EXECUTABLE)
  add_dependencies(lint_check reconfigure)
endif()
if(CLANG_TIDY_DIRECT_EXECUTABLE)
  add_dependencies(lint_fix reconfigure)
endif()

if(TARGET ${PROJECT_NAME})
  if(ENFORCE_FORMAT AND CLANG_FORMAT_EXECUTABLE)
    add_dependencies(${PROJECT_NAME} format_fix)
  endif()
  if(ENFORCE_LINT AND CLANG_TIDY_EXECUTABLE)
    add_dependencies(${PROJECT_NAME} lint_check)
  endif()
endif()

# =============================================================================
# Configuration Summary
# =============================================================================
message(STATUS "")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "Validation Tools Configuration")
message(STATUS "")
message(STATUS "Tools Found")
message(STATUS "  clang-format:       ${CLANG_FORMAT_EXECUTABLE}")
message(STATUS "  clang-tidy:         ${CLANG_TIDY_EXECUTABLE}")
message(STATUS "  clang-tidy (direct):${CLANG_TIDY_DIRECT_EXECUTABLE}")
message(STATUS "  gcovr:              ${GCOVR_EXECUTABLE}")
message(STATUS "")
message(STATUS "Validation Mode")
if(CI_MODE)
  message(STATUS "  Mode:   CI (all files)")
else()
  message(STATUS "  Mode:   Dev (changed files only)")
endif()
message(STATUS "  Format: ${format_count} files")
message(STATUS "  Lint:   ${lint_count} files")

if(ENFORCE_FORMAT OR ENFORCE_LINT)
  message(STATUS "")
  message(STATUS "Enforce on Build")
  message(STATUS "  Format: ${ENFORCE_FORMAT}")
  message(STATUS "  Lint:   ${ENFORCE_LINT}")
endif()

message(STATUS "")
message(STATUS "Available Targets")
if(CLANG_FORMAT_EXECUTABLE)
  message(STATUS "  format_check:  Check ${validation_mode} files formatting")
  message(STATUS "  format_fix:    Fix ${validation_mode} files formatting")
endif()
if(CLANG_TIDY_EXECUTABLE)
  message(STATUS "  lint_check:    Check ${validation_mode} files lint  (run_static_analysis.sh --check)")
endif()
if(CLANG_TIDY_DIRECT_EXECUTABLE)
  message(STATUS "  lint_fix:      Fix ${validation_mode} files lint    (run_static_analysis.sh -fix)")
endif()
if(TARGET validate)
  message(STATUS "  validate:      Run format_fix + lint_check")
endif()
if(GCOVR_EXECUTABLE AND ENABLE_COVERAGE)
  message(STATUS "  coverage:        Generate HTML + XML + Text coverage report")
  message(STATUS "  coverage_check:  Fail if line coverage < ${COVERAGE_TARGET_PERCENT}%")
  message(STATUS "  coverage_quick:  Print coverage summary (no file output)")
  message(STATUS "  verify_coverage: Verify coverage >= ${COVERAGE_TARGET_PERCENT}%")
endif()
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "")
