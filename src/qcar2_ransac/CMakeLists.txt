cmake_minimum_required(VERSION 3.8)
project(qcar2_ransac)

# C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(qcar2_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# C++ source files (shared by build target and quality targets)
set(QCAR2_RANSAC_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ransac_localizer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ransac_localizer_node.cpp
)

# RANSAC Localizer Node (C++)
add_executable(ransac_localizer_node ${QCAR2_RANSAC_SOURCES})

ament_target_dependencies(ransac_localizer_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  qcar2_msgs
)

target_link_libraries(ransac_localizer_node
  ${OpenCV_LIBRARIES}
)

# Install C++ targets
install(TARGETS
  ransac_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Python module
ament_python_install_package(${PROJECT_NAME})

# Install launch files used in default runtime flow
install(FILES
  launch/ransac_localizer_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# compile_commands.json symlink (for clangd IDE integration)
add_custom_target(compile_commands ALL
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_SOURCE_DIR}/compile_commands.json
  COMMENT "Linking compile_commands.json to source root for clangd..."
)

# Testing
if(BUILD_TESTING)
  # Catch2 via FetchContent (provides <catch2/...> headers and catch_discover_tests)
  include(FetchContent)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.7.1
  )
  FetchContent_MakeAvailable(Catch2)
  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
  include(CTest)
  include(Catch)

  # catch_ros2 provides ROS 2-aware test runner (main)
  find_package(catch_ros2 REQUIRED)

  # Unit test: NormalizeClassName
  add_executable(test_normalize_class_name
    test/unit/ransac_localizer/test_normalize_class_name.cpp
    src/ransac_localizer.cpp
  )
  FetchContent_GetProperties(Catch2
    SOURCE_DIR catch2_src_dir
    BINARY_DIR catch2_bin_dir)

  target_include_directories(test_normalize_class_name PRIVATE
    include
    $<BUILD_INTERFACE:${catch2_src_dir}/src>
    $<BUILD_INTERFACE:${catch2_bin_dir}/generated-includes>)
  ament_target_dependencies(test_normalize_class_name
    rclcpp sensor_msgs geometry_msgs visualization_msgs
    tf2 tf2_ros tf2_geometry_msgs cv_bridge image_transport qcar2_msgs
  )
  target_link_libraries(test_normalize_class_name
    catch_ros2::catch_ros2_with_node_main ${OpenCV_LIBRARIES}
  )
  catch_discover_tests(test_normalize_class_name)

  add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running tests via CTest..."
    DEPENDS test_normalize_class_name
  )
else()
  add_custom_target(run_tests
    COMMAND ${CMAKE_COMMAND} -E echo "BUILD_TESTING is OFF; no tests configured"
    COMMAND ${CMAKE_COMMAND} -E false
  )
endif()

# ---------------------------------------------------------------------------
# Code-quality targets (always available, no forced rebuild of main target)
# ---------------------------------------------------------------------------

# Helper: stub target that fails with a "tool not found" message
function(add_unavailable_target target_name tool_name)
  add_custom_target(${target_name}
    COMMAND ${CMAKE_COMMAND} -E echo "${tool_name} not found"
    COMMAND ${CMAKE_COMMAND} -E false
  )
endfunction()

# clang-format targets (header included — clang-format accepts it directly)
set(QCAR2_RANSAC_FORMAT_SOURCES
  ${QCAR2_RANSAC_SOURCES}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/qcar2_ransac/ransac_localizer.hpp
)

find_program(CLANG_FORMAT_EXE NAMES clang-format)
if(CLANG_FORMAT_EXE)
  add_custom_target(format_check
    COMMAND ${CLANG_FORMAT_EXE} --dry-run --Werror ${QCAR2_RANSAC_FORMAT_SOURCES}
    COMMENT "Checking code formatting with clang-format..."
  )
  add_custom_target(format_fix
    COMMAND ${CLANG_FORMAT_EXE} -i ${QCAR2_RANSAC_FORMAT_SOURCES}
    COMMENT "Fixing code formatting with clang-format..."
  )
else()
  message(STATUS "clang-format not found; format_check and format_fix targets will not be functional")
  add_unavailable_target(format_check clang-format)
  add_unavailable_target(format_fix clang-format)
endif()

# clang-tidy targets (.cpp only; headers are analysed transitively via compile_commands.json)
find_program(CLANG_TIDY_EXE NAMES clang-tidy)
if(CLANG_TIDY_EXE)
  add_custom_target(lint_check
    COMMAND ${CLANG_TIDY_EXE} -p ${CMAKE_BINARY_DIR} ${QCAR2_RANSAC_SOURCES}
    COMMENT "Running clang-tidy lint check..."
  )
  add_custom_target(lint_fix
    COMMAND ${CLANG_TIDY_EXE} -p ${CMAKE_BINARY_DIR} --fix ${QCAR2_RANSAC_SOURCES}
    COMMENT "Running clang-tidy lint fix..."
  )
else()
  message(STATUS "clang-tidy not found; lint_check and lint_fix targets will not be functional")
  add_unavailable_target(lint_check clang-tidy)
  add_unavailable_target(lint_fix clang-tidy)
endif()

ament_package()
