# =============================================================================
# Dependency Management
# =============================================================================

string(TOUPPER "${CMAKE_PROJECT_NAME}" PROJECT_NAME_PREFIX)
string(REPLACE "-" "_" PROJECT_NAME_PREFIX "${PROJECT_NAME_PREFIX}")

# =============================================================================
# Load Dependency Declarations
# =============================================================================
include("${CMAKE_CURRENT_SOURCE_DIR}/dependencies.cmake")

# =============================================================================
# Add Dependencies
# =============================================================================

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/externals/CMakeLists.txt")
  add_subdirectory(externals)
  set(HAS_EXTERNALS TRUE)
else()
  set(HAS_EXTERNALS FALSE)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/internals/CMakeLists.txt")
  add_subdirectory(internals)
  set(HAS_INTERNALS TRUE)
else()
  set(HAS_INTERNALS FALSE)
endif()

# =============================================================================
# Configuration Summary
# =============================================================================
message(STATUS "")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "Dependency Configuration for ${CMAKE_PROJECT_NAME}")
message(STATUS "")
message(STATUS "Dependency Modules")
message(STATUS "  External: ${HAS_EXTERNALS}")
message(STATUS "  Internal: ${HAS_INTERNALS}")

if(HAS_EXTERNALS OR HAS_INTERNALS)
  message(STATUS "")
  message(STATUS "Paths")
  if(HAS_EXTERNALS)
    message(STATUS "  Externals: ${CMAKE_CURRENT_SOURCE_DIR}/externals")
  endif()
  if(HAS_INTERNALS)
    message(STATUS "  Internals: ${CMAKE_CURRENT_SOURCE_DIR}/internals")
  endif()
endif()

message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "")
