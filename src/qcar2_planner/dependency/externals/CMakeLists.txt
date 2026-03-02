# =============================================================================
# External Dependencies Management
# =============================================================================
# Processes two dependency lists from dependencies.cmake:
#
# EXTERNAL_DEPENDENCIES (source, built via FetchContent)
# Format: "name|repo|tag|description|build_options|target_options"
#
# EXTERNAL_BINARY_DEPENDENCIES (pre-built archives, downloaded via FetchContent)
# Format: "name|url_template|version|description|hash"

include(FetchContent)

# =============================================================================
# Parse and Fetch External Dependency
# =============================================================================
function(process_external_dependency ENTRY)
  # Parse pipe-delimited entry
  string(REPLACE "|" ";" parts "${ENTRY}")
  list(LENGTH parts parts_len)

  list(GET parts 0 name)
  list(GET parts 1 repo)
  list(GET parts 2 tag)
  list(GET parts 3 desc)

  # Optional fields
  set(build_opts "")
  set(target_opts "")
  if(parts_len GREATER 4)
    list(GET parts 4 build_opts)
  endif()
  if(parts_len GREATER 5)
    list(GET parts 5 target_opts)
  endif()

  message(STATUS "  ${name}: ${desc}")

  # Set build options before FetchContent (comma-separated)
  if(build_opts)
    string(REPLACE "," ";" build_opts_list "${build_opts}")
    foreach(option IN LISTS build_opts_list)
      string(REPLACE " " ";" option_parts "${option}")
      list(GET option_parts 0 option_name)
      list(GET option_parts 1 option_value)

      # Detect option type: BOOL for boolean values, STRING otherwise
      string(TOUPPER "${option_value}" option_value_upper)
      if(option_value_upper MATCHES "^(ON|OFF|TRUE|FALSE|YES|NO|1|0)$")
        set(${option_name} ${option_value} CACHE BOOL "" FORCE)
      else()
        set(${option_name} ${option_value} CACHE STRING "" FORCE)
      endif()
    endforeach()
  endif()

  # Fetch the dependency
  FetchContent_Declare(
    ${name}
    GIT_REPOSITORY ${repo}
    GIT_TAG ${tag}
    GIT_SHALLOW TRUE
    SYSTEM
  )

  # Suppress deprecation warnings and exclude from compile_commands.json
  set(_saved_warn_deprecated "${CMAKE_WARN_DEPRECATED}")
  set(CMAKE_WARN_DEPRECATED OFF)
  set(_saved_ecc "${CMAKE_EXPORT_COMPILE_COMMANDS}")
  set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)
  FetchContent_MakeAvailable(${name})
  set(CMAKE_EXPORT_COMPILE_COMMANDS "${_saved_ecc}")
  set(CMAKE_WARN_DEPRECATED "${_saved_warn_deprecated}")

  # Apply target compile options
  if(target_opts)
    string(REPLACE " " ";" target_opts_list "${target_opts}")
    list(GET target_opts_list 0 target_name)
    list(REMOVE_AT target_opts_list 0)
    if(TARGET ${target_name})
      target_compile_options(${target_name} PRIVATE ${target_opts_list})
    endif()
  endif()

  # Call post-fetch hook if defined
  if(COMMAND dependency_post_fetch)
    dependency_post_fetch(${name})
    # Propagate CMAKE_MODULE_PATH changes
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)
  endif()
endfunction()

# =============================================================================
# Parse and Fetch External Binary Dependency
# =============================================================================
function(process_external_binary_dependency ENTRY)
  string(REPLACE "|" ";" parts "${ENTRY}")
  list(LENGTH parts parts_len)

  list(GET parts 0 name)
  list(GET parts 1 url_template)
  list(GET parts 2 version)
  list(GET parts 3 desc)

  set(hash_value "")
  if(parts_len GREATER 4)
    list(GET parts 4 hash_spec)
    string(TOUPPER "${hash_spec}" hash_spec_upper)
    if(hash_spec_upper MATCHES "^([A-Z0-9]+)=(.+)$")
      set(hash_algo "${CMAKE_MATCH_1}")
      set(hash_value "${CMAKE_MATCH_2}")
    endif()
  endif()

  message(STATUS "  ${name}: ${desc}")

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(platform_lower "linux")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(platform_lower "macos")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(platform_lower "windows")
  else()
    message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}")
  endif()

  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(arch "x86_64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(arch "arm64")
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()

  string(REPLACE "\${VERSION}" "${version}" resolved_url "${url_template}")
  string(REPLACE "\${PLATFORM_LOWER}" "${platform_lower}" resolved_url "${resolved_url}")
  string(REPLACE "\${ARCH}" "${arch}" resolved_url "${resolved_url}")

  if(hash_value)
    FetchContent_Declare(
      ${name}
      URL ${resolved_url}
      URL_HASH ${hash_algo}=${hash_value}
      SYSTEM
    )
  else()
    FetchContent_Declare(
      ${name}
      URL ${resolved_url}
      SYSTEM
    )
  endif()

  string(TOLOWER "${name}" name_lower)
  if(NOT ${name_lower}_POPULATED)
    # FetchContent_Populate(<name>) single-arg form is deprecated since CMake
    # 3.28. FetchContent_MakeAvailable cannot replace it here because it calls
    # add_subdirectory on the archive, conflicting with the manual IMPORTED
    # target created below. CMP0169 (introduced in CMake 3.30) explicitly
    # preserves the old behaviour when set to OLD.
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.30")
      cmake_policy(SET CMP0169 OLD)
    endif()
    FetchContent_Populate(${name})
  endif()
  set(source_dir "${${name_lower}_SOURCE_DIR}")

  # Detect wrapped or flat archive layout
  set(include_dir "")
  set(lib_dir "")
  if(EXISTS "${source_dir}/${name}/include")
    set(include_dir "${source_dir}/${name}/include")
    set(lib_dir "${source_dir}/${name}/lib")
  elseif(EXISTS "${source_dir}/include")
    set(include_dir "${source_dir}/include")
    set(lib_dir "${source_dir}/lib")
  else()
    message(FATAL_ERROR "Cannot find include directory in ${source_dir}")
  endif()

  string(TOUPPER "${name}" name_upper)
  find_library(
    ${name_upper}_LIBRARY
    NAMES ${name}
    PATHS ${lib_dir}
    NO_DEFAULT_PATH
    REQUIRED
  )

  # Bare target required as the real IMPORTED target backing the alias below.
  # Consumers should link against ${name}::${name} for namespaced access.
  add_library(${name} STATIC IMPORTED GLOBAL)
  set_target_properties(${name} PROPERTIES
    IMPORTED_LOCATION "${${name_upper}_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${include_dir}"
  )

  add_library(${name}::${name} ALIAS ${name})

  if(COMMAND dependency_post_fetch)
    dependency_post_fetch(${name})
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)
  endif()
endfunction()

# =============================================================================
# Configuration Summary
# =============================================================================
message(STATUS "")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "External Dependencies Configuration")
message(STATUS "")
message(STATUS "Source Dependencies")

# Process all source dependencies
list(LENGTH EXTERNAL_DEPENDENCIES dep_count)
if(dep_count EQUAL 0)
  message(STATUS "  None configured")
else()
  foreach(dep IN LISTS EXTERNAL_DEPENDENCIES)
    process_external_dependency("${dep}")
  endforeach()
endif()

# Handle Tracy disabled message
if(NOT ENABLE_PROFILING)
  message(STATUS "  Tracy: Disabled (ENABLE_PROFILING=OFF)")
endif()

message(STATUS "")
message(STATUS "Binary Dependencies")

# Process all binary dependencies
list(LENGTH EXTERNAL_BINARY_DEPENDENCIES bin_dep_count)
if(bin_dep_count EQUAL 0)
  message(STATUS "  None configured")
else()
  foreach(dep IN LISTS EXTERNAL_BINARY_DEPENDENCIES)
    process_external_binary_dependency("${dep}")
  endforeach()
endif()

message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "")
