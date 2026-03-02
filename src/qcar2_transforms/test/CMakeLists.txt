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

include(${PROJECT_SOURCE_DIR}/ament/test/tests.cmake)

process_test_type(integration
  SOURCES
    integration/test_package.py
)

add_subdirectory(unit)

# =============================================================================
# Run Tests Target
# =============================================================================
add_custom_target(run_tests
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure --verbose
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  COMMENT "Running all registered tests via CTest..."
  VERBATIM
)

# =============================================================================
# Coverage Targets (only available when ENABLE_COVERAGE=ON)
# =============================================================================
if(ENABLE_COVERAGE)
  if(NOT "${COVERAGE_TARGET_PERCENT}" MATCHES "^[0-9]+$")
    message(FATAL_ERROR
      "COVERAGE_TARGET_PERCENT must be a non-negative integer, "
      "got: '${COVERAGE_TARGET_PERCENT}'"
    )
  endif()

  set(PYTEST_COV_ARGS
    ${CMAKE_COMMAND} -E env PYTHONDONTWRITEBYTECODE=1
    python3 -m pytest
    "--cov=${PROJECT_SOURCE_DIR}/${PROJECT_NAME}"
    "--cov-branch"
    "--cov-report=term-missing"
    "${PROJECT_SOURCE_DIR}/test/"
  )

  add_custom_target(coverage
    COMMAND ${PYTEST_COV_ARGS}
      --cov-report=html:${CMAKE_BINARY_DIR}/coverage/htmlcov
      --cov-report=xml:${CMAKE_BINARY_DIR}/coverage/coverage.xml
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    COMMENT "Running pytest with coverage report generation..."
    COMMAND_EXPAND_LISTS
    VERBATIM
  )

  add_custom_target(coverage-check
    COMMAND ${PYTEST_COV_ARGS}
      --cov-fail-under=${COVERAGE_TARGET_PERCENT}
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    COMMENT "Checking coverage threshold (${COVERAGE_TARGET_PERCENT}%)..."
    COMMAND_EXPAND_LISTS
    VERBATIM
  )
endif()

# =============================================================================
# Configuration Summary
# =============================================================================
message(STATUS "")
message(STATUS "  Test root: ${PROJECT_TEST_PATH}")
message(STATUS "")
message(STATUS "Coverage")
message(STATUS "  Enabled: ${ENABLE_COVERAGE} (target: ${COVERAGE_TARGET_PERCENT}%)")
message(STATUS "")
