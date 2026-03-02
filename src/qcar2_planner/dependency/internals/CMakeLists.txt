# =============================================================================
# Internal Dependencies Management
# =============================================================================
# Processes INTERNAL_DEPENDENCIES variable from dependencies.cmake
# Format: "name|repo|tag|description|build_options|target_options"
#
# Unlike externals:
#   - Source is placed in dependency/internals/<name> (not _deps/)
#   - NOT marked as SYSTEM (clang-tidy/format applies)
#   - Can be modified locally for debugging/customization
#   - Git tracked for co-development
#   - Auto-deleted when removed from dependencies.cmake

include(FetchContent)

# =============================================================================
# Collect Declared Internal Dependency Names
# =============================================================================
set(declared_internals "")
foreach(dep IN LISTS INTERNAL_DEPENDENCIES)
  string(REPLACE "|" ";" parts "${dep}")
  list(GET parts 0 name)
  list(APPEND declared_internals ${name})
endforeach()

# =============================================================================
# Auto-delete Undeclared Internal Dependencies
# =============================================================================
file(GLOB existing_items "${CMAKE_CURRENT_SOURCE_DIR}/*")
foreach(item ${existing_items})
  if(IS_DIRECTORY ${item})
    get_filename_component(dir_name ${item} NAME)
    # Skip if it's a declared dependency
    if(NOT dir_name IN_LIST declared_internals)
      message(STATUS "  Removing undeclared internal dependency: ${dir_name}")
      file(REMOVE_RECURSE ${item})
    endif()
  endif()
endforeach()

# =============================================================================
# Parse and Fetch Internal Dependency
# =============================================================================
function(process_internal_dependency ENTRY)
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
      set(${option_name} ${option_value} CACHE BOOL "" FORCE)
    endforeach()
  endif()

  # Fetch the dependency to internals directory
  FetchContent_Declare(
    ${name}
    GIT_REPOSITORY ${repo}
    GIT_TAG ${tag}
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/dependency/internals/${name}
    GIT_SHALLOW TRUE
    # No SYSTEM option - lint/format applies to internal dependencies
  )

  FetchContent_MakeAvailable(${name})

  # Apply target compile options
  if(target_opts)
    string(REPLACE " " ";" target_opts_list "${target_opts}")
    list(GET target_opts_list 0 target_name)
    list(REMOVE_AT target_opts_list 0)
    if(TARGET ${target_name})
      target_compile_options(${target_name} PRIVATE ${target_opts_list})
    endif()
  endif()
endfunction()

# =============================================================================
# Configuration Summary
# =============================================================================
message(STATUS "")
message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "Internal Dependencies Configuration")
message(STATUS "")
message(STATUS "Dependencies")

# Process all internal dependencies
list(LENGTH INTERNAL_DEPENDENCIES dep_count)
if(dep_count EQUAL 0)
  message(STATUS "  None configured")
else()
  foreach(dep IN LISTS INTERNAL_DEPENDENCIES)
    process_internal_dependency("${dep}")
  endforeach()
endif()

message(STATUS "-----------------------------------------------------------------------------")
message(STATUS "")
