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

# Catch2 v3 compatible test discovery script.
# Replaces CatchAddTests.cmake (which uses the Catch v2 CLI flag
# --list-test-names-only). Catch v3 uses --list-tests --verbosity quiet.
#
# Called by catch_discover_tests() POST_BUILD step with the same -D variables
# as the upstream CatchAddTests.cmake, so it is a drop-in replacement.

set(prefix "${TEST_PREFIX}")
set(suffix "${TEST_SUFFIX}")
set(spec ${TEST_SPEC})
set(extra_args ${TEST_EXTRA_ARGS})
set(properties ${TEST_PROPERTIES})
set(reporter ${TEST_REPORTER})
set(output_dir ${TEST_OUTPUT_DIR})
set(output_prefix ${TEST_OUTPUT_PREFIX})
set(output_suffix ${TEST_OUTPUT_SUFFIX})
set(script)
set(tests)

function(add_command NAME)
  set(_args "")
  math(EXPR _last_arg ${ARGC}-1)
  foreach(_n RANGE 1 ${_last_arg})
    set(_arg "${ARGV${_n}}")
    if(_arg MATCHES "[^-./:a-zA-Z0-9_]")
      set(_args "${_args} [==[${_arg}]==]")
    else()
      set(_args "${_args} ${_arg}")
    endif()
  endforeach()
  set(script "${script}${NAME}(${_args})\n" PARENT_SCOPE)
endfunction()

if(NOT EXISTS "${TEST_EXECUTABLE}")
  message(FATAL_ERROR
    "Specified test executable '${TEST_EXECUTABLE}' does not exist"
  )
endif()

# Catch2 v3: --list-tests --verbosity quiet prints one test name per line.
execute_process(
  COMMAND ${TEST_EXECUTOR} "${TEST_EXECUTABLE}" ${spec}
          --list-tests --verbosity quiet
  OUTPUT_VARIABLE output
  ERROR_VARIABLE  error_output
  RESULT_VARIABLE result
  WORKING_DIRECTORY "${TEST_WORKING_DIR}"
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT result EQUAL 0)
  message(FATAL_ERROR
    "Error running test executable '${TEST_EXECUTABLE}':\n"
    "  Result: ${result}\n"
    "  Output: ${output}\n"
    "  Error: ${error_output}\n"
  )
endif()

if(output STREQUAL "")
  message(WARNING
    "Test executable '${TEST_EXECUTABLE}' reported no tests.\n"
  )
endif()

string(REPLACE "\n" ";" output "${output}")

foreach(line IN LISTS output)
  string(STRIP "${line}" test)
  if(test STREQUAL "")
    continue()
  endif()

  # Escape characters that Catch2 treats as special in test-name filters.
  set(test_name "${test}")
  foreach(char , [ ])
    string(REPLACE "${char}" "\\${char}" test_name "${test_name}")
  endforeach()

  if(output_dir)
    if(NOT IS_ABSOLUTE "${output_dir}")
      set(output_dir "${TEST_WORKING_DIR}/${output_dir}")
    endif()
    string(REGEX REPLACE "[^A-Za-z0-9_]" "_" test_name_clean "${test_name}")
    set(output_dir_arg "--out" "${output_dir}/${output_prefix}${test_name_clean}${output_suffix}")
  else()
    set(output_dir_arg "")
  endif()

  if(reporter)
    set(reporter_arg "--reporter" "${reporter}")
  else()
    set(reporter_arg "")
  endif()

  add_command(add_test
    "${prefix}${test}${suffix}"
    ${TEST_EXECUTOR}
    "${TEST_EXECUTABLE}"
    "${test_name}"
    ${extra_args}
    ${reporter_arg}
    ${output_dir_arg}
  )
  add_command(set_tests_properties
    "${prefix}${test}${suffix}"
    PROPERTIES
      WORKING_DIRECTORY "${TEST_WORKING_DIR}"
      ${properties}
  )
  list(APPEND tests "${prefix}${test}${suffix}")
endforeach()

add_command(set "${TEST_LIST}" ${tests})

file(WRITE "${CTEST_FILE}" "${script}")
