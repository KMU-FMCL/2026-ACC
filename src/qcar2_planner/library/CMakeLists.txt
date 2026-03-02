# =============================================================================
# Library Configuration
# =============================================================================

# =============================================================================
# Source Files
# =============================================================================
set(LIBRARY_SOURCES
  source/core/bernstein/polynomial.cpp
  source/core/hodograph/planar_point.cpp
  source/core/hodograph/quintic.cpp
  source/core/rbf/gaussian.cpp
)

# Public dependencies (exposed in public API headers)
set(LIBRARY_DEPENDENCIES_PUBLIC
  playground
)

# Private dependencies (implementation only, not exposed in headers)
set(LIBRARY_DEPENDENCIES_PRIVATE
  Eigen3::Eigen
)

# =============================================================================
# Create Library Target
# =============================================================================
add_library(${PROJECT_NAME} SHARED
  ${LIBRARY_SOURCES}
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# =============================================================================
# Target Properties
# =============================================================================
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_INCLUDE_PATH}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${LIBRARY_DEPENDENCIES_PUBLIC}
  PRIVATE
    ${LIBRARY_DEPENDENCIES_PRIVATE}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall -Wextra -Wpedantic -Werror
  )
endif()

if(ENABLE_COVERAGE)
  target_compile_options(${PROJECT_NAME} PRIVATE --coverage -g -O0)
  target_link_options(${PROJECT_NAME} PRIVATE --coverage)
endif()

# =============================================================================
# Installation Rules
# =============================================================================
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# =============================================================================
# Configuration Summary
# =============================================================================
message(STATUS "")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "Library Configuration for ${PROJECT_NAME}")
message(STATUS "")
message(STATUS "Library Settings")
message(STATUS "  Target Name:   ${PROJECT_NAME}")
message(STATUS "  Library Type:  SHARED")
message(STATUS "  Include Path:  ${CMAKE_CURRENT_SOURCE_DIR}/include")
message(STATUS "  Source Files:  ${LIBRARY_SOURCES}")
message(STATUS "")
message(STATUS "Dependencies")
message(STATUS "  Public:   ${LIBRARY_DEPENDENCIES_PUBLIC}")
message(STATUS "  Private:  ${LIBRARY_DEPENDENCIES_PRIVATE}")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "")
