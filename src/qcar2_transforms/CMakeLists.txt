cmake_minimum_required(VERSION 3.25)

# =============================================================================
# Project Definition
# =============================================================================
project(qcar2_transforms VERSION 0.1.0 LANGUAGES NONE)

# =============================================================================
# Build Options
# =============================================================================
option(ENABLE_COVERAGE "Enable Python test coverage measurement" OFF)
set(COVERAGE_TARGET_PERCENT "80" CACHE STRING "Minimum line coverage percentage")

# =============================================================================
# Project Paths
# =============================================================================
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})
set(PROJECT_SOURCE_PATH ${PROJECT_ROOT}/qcar2_transforms)
set(PROJECT_TEST_PATH ${PROJECT_ROOT}/test)
set(PROJECT_CONFIG_PATH ${PROJECT_ROOT}/config)

# =============================================================================
# ROS 2 Dependencies
# =============================================================================
include(ament/dependency/imports.cmake)

# =============================================================================
# Python Module
# =============================================================================
include(ament/dependency/exports.cmake)

# =============================================================================
# Subdirectories
# =============================================================================
add_subdirectory(config)

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
message(STATUS "  Name:    ${PROJECT_NAME}")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "")
message(STATUS "Build Options")
message(STATUS "  Coverage: ${ENABLE_COVERAGE} (target: ${COVERAGE_TARGET_PERCENT}%)")
message(STATUS "")
message(STATUS "Paths")
message(STATUS "  Root:   ${PROJECT_ROOT}")
message(STATUS "  Source: ${PROJECT_SOURCE_PATH}")
message(STATUS "  Test:   ${PROJECT_TEST_PATH}")
message(STATUS "  Config: ${PROJECT_CONFIG_PATH}")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "")

# =============================================================================
# Package Export
# =============================================================================
ament_package()
