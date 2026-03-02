# =============================================================================
# Dependency Declarations
# =============================================================================
# Single source of truth for all project dependencies.
# Format: "name|repo|tag|description|build_options|target_options"
#
# build_options: comma-separated "OPTION VALUE" pairs
# target_options: "target compile_option1 compile_option2 ..."
# =============================================================================

# =============================================================================
# External Dependencies (read-only, SYSTEM marked, _deps/)
# =============================================================================
# - spdlog: Fast C++ logging library
# - fkYAML: Lightweight YAML parser
# - Catch2: Testing framework
# - FakeIt: C++ mocking framework
# - doxygen-awesome-css: Modern Doxygen theme
# - Eigen3: C++ template library for linear algebra
set(EXTERNAL_DEPENDENCIES
  "spdlog|https://github.com/gabime/spdlog.git|v1.14.1|Fast C++ logging library|SPDLOG_BUILD_TESTS OFF,SPDLOG_BUILD_EXAMPLE OFF,SPDLOG_BUILD_BENCH OFF|"
  "fkYAML|https://github.com/fktn-k/fkYAML.git|v0.3.12|Lightweight YAML parser|FK_YAML_BUILD_TEST OFF|"
  "Catch2|https://github.com/catchorg/Catch2.git|v3.5.2|Testing framework|CATCH_BUILD_TESTING OFF,CATCH_INSTALL_DOCS OFF,CATCH_INSTALL_EXTRAS OFF|"
  "FakeIt|https://github.com/eranpeer/FakeIt.git|2.5.0|C++ mocking framework|ENABLE_TESTING OFF|"
  "doxygen-awesome-css|https://github.com/jothepro/doxygen-awesome-css.git|v2.3.4|Modern Doxygen theme||"
  "eigen3|https://gitlab.com/libeigen/eigen.git|5.0.0|C++ linear algebra library|EIGEN_BUILD_TESTING OFF,EIGEN_BUILD_DOC OFF|"
)

# =============================================================================
# Conditional External Dependencies
# =============================================================================
# - tracy: Real-time frame profiler (requires ENABLE_PROFILING=ON)
if(ENABLE_PROFILING)
  list(APPEND EXTERNAL_DEPENDENCIES
    "tracy|https://github.com/wolfpld/tracy.git|v0.11.1|Real-time frame profiler|TRACY_ENABLE ON,TRACY_ON_DEMAND OFF|TracyClient -Wno-error -w"
  )
endif()

# =============================================================================
# External Binary Dependencies (GitHub Releases, SYSTEM marked, _deps/)
# =============================================================================
# Format: "name|url_template|version|description|hash"
#
# url_template variables: ${VERSION}, ${PLATFORM_LOWER}, ${ARCH}
# hash: Optional but recommended. Format: ALGORITHM=value (e.g., SHA256=abc123...)
#       Verifies archive integrity during download
#
# - Playground: HolyGround production C++ core library
set(EXTERNAL_BINARY_DEPENDENCIES
  "playground|https://github.com/taehun-kmu/playground-cpp-release/releases/download/\${VERSION}/playground-\${VERSION}-\${PLATFORM_LOWER}-\${ARCH}.tar.gz|v0.1.0|HolyGround production C++ core library|SHA256=ecd0ccf1dc1993f02abfed6848a5a18cb7b252e3d55e925e9e1db739da3d228d"
)

# =============================================================================
# Internal Dependencies (modifiable, NOT SYSTEM, dependency/internals/)
# =============================================================================
# Use for libraries requiring local modifications or debugging.
# These are git-tracked for co-development.
# Example:
#   "fmt|https://github.com/fmtlib/fmt.git|11.2.0|Modern formatting library|FMT_TEST OFF,FMT_DOC OFF|"
set(INTERNAL_DEPENDENCIES
)

# =============================================================================
# Post-fetch Hooks
# =============================================================================
# Special handling required after fetching certain dependencies.
# Called by externals/CMakeLists.txt after processing.
function(dependency_post_fetch NAME)
  if(NAME STREQUAL "Catch2")
    # Add Catch2 extras to module path for CTest integration
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)
  endif()
endfunction()
