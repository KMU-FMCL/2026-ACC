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

find_package(ament_cmake_pytest REQUIRED)

# =============================================================================
# Function: normalize_test_name
# =============================================================================
# Converts a source file path to a normalized test name component.
# Example: "core/frame_transform/test_rotation.py" -> "core__frame_transform__test_rotation"
function(normalize_test_name SOURCE_PATH OUTPUT_VAR)
  string(REGEX REPLACE "\\.py$" "" name "${SOURCE_PATH}")
  string(REPLACE "/" "__" name "${name}")
  set(${OUTPUT_VAR} "${name}" PARENT_SCOPE)
endfunction()

# =============================================================================
# Function: add_pytest
# =============================================================================
# Registers a pytest test with PYTHONDONTWRITEBYTECODE=1 injected.
# Workaround for ROS 2 Humble lacking the upstream ament_cmake PR #533 fix.
function(add_pytest testname path)
  ament_add_pytest_test(${testname} ${path}
    ENV PYTHONDONTWRITEBYTECODE=1
    ${ARGN}
  )
endfunction()

# =============================================================================
# Function: process_test_type
# =============================================================================
# Registers a list of pytest files for a given test type.
# Parameters:
#   TEST_TYPE - Type name (unit, integration, etc.)
#   SOURCES   - Paths relative to the calling CMakeLists.txt directory
# Test name follows: ${PROJECT_NAME}__test__${TEST_TYPE}__${normalized_path}
function(process_test_type TEST_TYPE)
  cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})
  foreach(source IN LISTS ARG_SOURCES)
    normalize_test_name("${source}" test_path)
    set(test_name "${PROJECT_NAME}__test__${TEST_TYPE}__${test_path}")
    add_pytest(${test_name} ${CMAKE_CURRENT_SOURCE_DIR}/${source})
  endforeach()
endfunction()
