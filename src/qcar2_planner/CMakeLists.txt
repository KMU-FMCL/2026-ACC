cmake_minimum_required(VERSION 3.25)

# =============================================================================
# Project Definition
# =============================================================================
project(qcar2_planner VERSION 0.1.0 LANGUAGES CXX)

# =============================================================================
# Build Options
# =============================================================================
option(ENABLE_COVERAGE "Enable code coverage instrumentation" OFF)
option(ENABLE_PROFILING "Enable Tracy frame profiler" OFF)
option(ENFORCE_FORMAT "Run clang-format before build" OFF)
option(ENFORCE_LINT "Run clang-tidy before build" OFF)
option(CI_MODE "Validate all files (OFF = changed files only)" OFF)

set(COVERAGE_TARGET_PERCENT "80" CACHE STRING "Target coverage percentage")

# =============================================================================
# Compiler Settings
# =============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Platform Detection
# =============================================================================
if(UNIX)
  set(platform_name "Linux")
else()
  message(FATAL_ERROR "Unsupported platform")
endif()

# =============================================================================
# Install Paths
# =============================================================================
include(GNUInstallDirs)

# =============================================================================
# Project Paths
# =============================================================================
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})
set(PROJECT_LIBRARY_PATH ${PROJECT_ROOT}/library)
set(PROJECT_SOURCE_PATH ${PROJECT_LIBRARY_PATH}/source)
set(PROJECT_INCLUDE_PATH ${PROJECT_LIBRARY_PATH}/include)
set(PROJECT_TEST_PATH ${PROJECT_ROOT}/test)

# =============================================================================
# Project Dependencies
# =============================================================================
add_subdirectory(dependency)

# =============================================================================
# ROS 2 Dependencies
# =============================================================================
include(ament/dependency/imports.cmake)

# =============================================================================
# Subdirectories
# =============================================================================
add_subdirectory(library)

# =============================================================================
# compile_commands.json Symlink (for clangd IDE integration)
# =============================================================================
add_custom_target(compile_commands ALL
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${PROJECT_ROOT}/compile_commands.json
  COMMENT "Linking compile_commands.json to source root for clangd..."
)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(validation)
  add_subdirectory(test)
endif()

# =============================================================================
# Configuration Summary
# =============================================================================
message(STATUS "")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "Project Configuration for ${PROJECT_NAME}")
message(STATUS "")
message(STATUS "Project")
message(STATUS "  Name:     ${PROJECT_NAME}")
message(STATUS "  Version:  ${PROJECT_VERSION}")
message(STATUS "  Platform: ${platform_name}")
message(STATUS "")
message(STATUS "C++ Settings")
message(STATUS "  Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "")
message(STATUS "Build Options")
message(STATUS "  Coverage:       ${ENABLE_COVERAGE} (target: ${COVERAGE_TARGET_PERCENT}%)")
message(STATUS "  Profiling:      ${ENABLE_PROFILING}")
message(STATUS "  Enforce Format: ${ENFORCE_FORMAT}")
message(STATUS "  Enforce Lint:   ${ENFORCE_LINT}")
message(STATUS "  CI Mode:        ${CI_MODE}")
message(STATUS "")
message(STATUS "Paths")
message(STATUS "  Root:    ${PROJECT_ROOT}")
message(STATUS "  Library: ${PROJECT_LIBRARY_PATH}")
message(STATUS "  Include: ${PROJECT_INCLUDE_PATH}")
message(STATUS "  Test:    ${PROJECT_TEST_PATH}")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "")

# =============================================================================
# Package Export
# =============================================================================
include(ament/dependency/exports.cmake)
