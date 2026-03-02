# Copyright (c) 2026 Taehun Jung
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

if(NOT BUILD_TESTING)
  return()
endif()

include("${PROJECT_ROOT}/ament/test/tests.cmake")
include("${PROJECT_ROOT}/ament/test/discovery.cmake")
set(_CATCH_DISCOVER_TESTS_SCRIPT
  "${PROJECT_ROOT}/ament/test/add_tests.cmake"
  CACHE INTERNAL "" FORCE)

set(TEST_TYPES unit integration)

# Convert file path to CMake target name component
# Input:  "qcar2_planner/example.cpp"
# Output: "qcar2_planner__example"
function(normalize_test_name SOURCE_PATH OUTPUT_VAR)
  string(REGEX REPLACE "\\.cpp$" "" name "${SOURCE_PATH}")
  string(REPLACE "/" "__" name "${name}")
  set(${OUTPUT_VAR} "${name}" PARENT_SCOPE)
endfunction()

# Create a single test executable with standard configuration
function(create_test_executable TARGET)
  cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})
  add_executable(${TARGET} ${ARG_SOURCES})

  FetchContent_GetProperties(Catch2
    SOURCE_DIR catch2_src_dir
    BINARY_DIR catch2_bin_dir)

  target_include_directories(${TARGET} PRIVATE
    $<BUILD_INTERFACE:${PROJECT_INCLUDE_PATH}>
    $<BUILD_INTERFACE:${catch2_src_dir}/src>
    $<BUILD_INTERFACE:${catch2_bin_dir}/generated-includes>)

  target_link_libraries(${TARGET}
    PRIVATE
      ${PROJECT_NAME}
      catch_ros2::catch_ros2_with_node_main)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(${TARGET} PRIVATE
      -Wall -Wextra -Wpedantic -Werror
    )
  endif()

  if(ENABLE_COVERAGE)
    target_compile_options(${TARGET} PRIVATE --coverage -g -O0)
    target_link_options(${TARGET} PRIVATE --coverage)
  endif()

  catch_discover_tests(${TARGET})
endfunction()

# Process all source files for a given test type directory
function(process_test_type TEST_TYPE)
  cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})
  foreach(SOURCE ${ARG_SOURCES})
    normalize_test_name("${SOURCE}" NORM_NAME)
    set(TARGET "${PROJECT_NAME}__test__${TEST_TYPE}__${NORM_NAME}")
    create_test_executable(${TARGET} SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE}")
  endforeach()
endfunction()

foreach(TEST_TYPE ${TEST_TYPES})
  add_subdirectory(${TEST_TYPE})
endforeach()

# =============================================================================
# Test Runner Target
# =============================================================================
add_custom_target(run_tests
  COMMAND ${CMAKE_CTEST_COMMAND} --test-dir ${CMAKE_BINARY_DIR} --output-on-failure
  COMMENT "Running all tests..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# =============================================================================
# Configuration Summary
# =============================================================================
message(STATUS "")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "Test Configuration for ${PROJECT_NAME}")
message(STATUS "")
message(STATUS "Test Settings")
message(STATUS "  Framework:    catch_ros2")
message(STATUS "  Types:        ${TEST_TYPES}")
message(STATUS "  Include Path: ${PROJECT_INCLUDE_PATH}")
message(STATUS "")
message(STATUS "Coverage")
message(STATUS "  Enabled: ${ENABLE_COVERAGE}")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "")
