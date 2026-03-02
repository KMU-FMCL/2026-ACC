# Copyright 2026 QCar2 Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

include(${CMAKE_CURRENT_LIST_DIR}/../ament/validation/validations.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../ament/test/tests.cmake)

ament_copyright()
ament_lint_cmake()
ament_flake8("${PROJECT_SOURCE_DIR}")
ament_pep257("${PROJECT_SOURCE_DIR}")

find_program(AMENT_FLAKE8_BIN NAMES "ament_flake8" REQUIRED)
if(AMENT_FLAKE8_BIN)
  add_custom_target(lint_check
    COMMAND ${AMENT_FLAKE8_BIN} "${PROJECT_SOURCE_DIR}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    COMMENT "Running flake8 on ${PROJECT_NAME}"
    VERBATIM
  )
endif()

find_program(AMENT_PEP257_BIN NAMES "ament_pep257" REQUIRED)
if(AMENT_PEP257_BIN)
  add_custom_target(format_check
    COMMAND ${AMENT_PEP257_BIN} "${PROJECT_SOURCE_DIR}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    COMMENT "Running pep257 on ${PROJECT_NAME}"
    VERBATIM
  )
endif()

find_program(AUTOFLAKE_BIN NAMES "autoflake")
find_program(AUTOPEP8_BIN NAMES "autopep8")
if(AUTOFLAKE_BIN AND AUTOPEP8_BIN)
  add_custom_target(lint_fix
    COMMENT "Auto-fixing flake8 violations (autoflake + autopep8)..."
    COMMAND ${AUTOFLAKE_BIN}
      --in-place --recursive
      --remove-all-unused-imports
      --remove-unused-variables
      "${PROJECT_SOURCE_DIR}"
    COMMAND ${AUTOPEP8_BIN}
      --in-place --recursive
      --max-line-length 99
      "${PROJECT_SOURCE_DIR}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    VERBATIM
  )
endif()

find_program(DOCFORMATTER_BIN NAMES "docformatter")
if(DOCFORMATTER_BIN)
  add_custom_target(format_fix
    COMMENT "Auto-fixing pep257 docstring violations (docformatter)..."
    COMMAND ${DOCFORMATTER_BIN}
      --in-place --recursive
      --wrap-summaries 99
      --wrap-descriptions 92
      "${PROJECT_SOURCE_DIR}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    VERBATIM
  )
endif()

# ==========================================================================
# Coverage Targets (only available when ENABLE_COVERAGE=ON)
# Note: uses pytest-cov (Python); gcovr is not used (C++ only).
# ==========================================================================
# =============================================================================
# Configuration Summary
# =============================================================================
message(STATUS "")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "Validation Configuration for ${PROJECT_NAME}")
message(STATUS "")
message(STATUS "Linters")
message(STATUS "  copyright:  enabled")
message(STATUS "  lint_cmake: enabled")
message(STATUS "  flake8:     ${AMENT_FLAKE8_BIN}")
message(STATUS "  pep257:     ${AMENT_PEP257_BIN}")
message(STATUS "")
message(STATUS "Fix Targets")
message(STATUS "  lint_fix:   autoflake + autopep8 (max-line-length 99)")
message(STATUS "  format_fix: docformatter (wrap-summaries 99)")
message(STATUS "")
message(STATUS "Coverage")
message(STATUS "  Enabled: ${ENABLE_COVERAGE} (target: ${COVERAGE_TARGET_PERCENT}%)")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "")

if(ENABLE_COVERAGE)
  if(NOT "${COVERAGE_TARGET_PERCENT}" MATCHES "^[0-9]+$")
    message(FATAL_ERROR
      "COVERAGE_TARGET_PERCENT must be a non-negative integer, "
      "got: '${COVERAGE_TARGET_PERCENT}'"
    )
  endif()

  set(PYTEST_COV_ARGS
    python3 -m pytest
    "--cov=${PROJECT_SOURCE_DIR}/${PROJECT_NAME}"
    "--cov-branch"
    "${PROJECT_SOURCE_DIR}/test/"
  )

  # coverage: run tests and generate HTML + XML report
  add_custom_target(coverage
    COMMAND ${PYTEST_COV_ARGS}
      --cov-report=term-missing
      --cov-report=html:${CMAKE_BINARY_DIR}/coverage/htmlcov
      --cov-report=xml:${CMAKE_BINARY_DIR}/coverage/coverage.xml
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    COMMENT "Running pytest with coverage report generation..."
    VERBATIM
  )

  # coverage-check: enforce threshold (non-zero exit if below target)
  add_custom_target(coverage-check
    COMMAND ${PYTEST_COV_ARGS}
      --cov-report=term-missing
      --cov-fail-under=${COVERAGE_TARGET_PERCENT}
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    COMMENT "Checking coverage threshold (${COVERAGE_TARGET_PERCENT}%)..."
    VERBATIM
  )
endif()
