cmake_minimum_required(VERSION 3.8)
project(qcar2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(qcar2_msgs REQUIRED)
find_package(qcar2_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/bezier_path_publisher
  scripts/ph_path_publisher
  scripts/dashboard
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(vfg_guidance_node src/vfg_guidance.cpp)
ament_target_dependencies(vfg_guidance_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  qcar2_msgs
  qcar2_interfaces
)
target_include_directories(vfg_guidance_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(pid_lateral_controller_node src/pid_controller.cpp)
ament_target_dependencies(pid_lateral_controller_node
  rclcpp
  qcar2_msgs
  qcar2_interfaces
)
target_include_directories(pid_lateral_controller_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_library(ph_runtime STATIC
  src/ph_runtime.cpp
)
ament_target_dependencies(ph_runtime
  qcar2_msgs
)
target_include_directories(ph_runtime PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(vfg_guidance_ph_node src/vfg_guidance_ph.cpp)
ament_target_dependencies(vfg_guidance_ph_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  qcar2_msgs
  qcar2_interfaces
)
target_include_directories(vfg_guidance_ph_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(vfg_guidance_ph_node
  ph_runtime
)

add_executable(pid_lateral_controller_ph_node src/pid_controller_ph.cpp)
ament_target_dependencies(pid_lateral_controller_ph_node
  rclcpp
  qcar2_msgs
  qcar2_interfaces
)
target_include_directories(pid_lateral_controller_ph_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(pid_lateral_controller_ph_node
  ph_runtime
)

install(TARGETS
  vfg_guidance_node
  pid_lateral_controller_node
  vfg_guidance_ph_node
  pid_lateral_controller_ph_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # ==========================================================================
  # Linters (individual)
  # ==========================================================================
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_cppcheck(LANGUAGE "c++"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_uncrustify()
  ament_xmllint()

  # ==========================================================================
  # C++ Source File List (shared by format and lint targets)
  # ==========================================================================
  file(GLOB_RECURSE cpp_sources CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  )

  # ==========================================================================
  # Format Targets (ament_uncrustify for C++)
  # ==========================================================================
  find_program(AMENT_UNCRUSTIFY_EXECUTABLE NAMES ament_uncrustify)

  if(AMENT_UNCRUSTIFY_EXECUTABLE)
    add_custom_target(format_check
      COMMAND ${AMENT_UNCRUSTIFY_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Checking C++ code formatting..."
      VERBATIM
    )

    add_custom_target(format_fix
      COMMAND ${AMENT_UNCRUSTIFY_EXECUTABLE} --reformat
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Fixing C++ code formatting..."
      VERBATIM
    )
  endif()

  # ==========================================================================
  # Lint Targets (clang-tidy for C++)
  # ==========================================================================
  find_program(CLANG_TIDY_EXECUTABLE NAMES clang-tidy-18 clang-tidy)

  if(CLANG_TIDY_EXECUTABLE)
    add_custom_target(lint_check
      COMMAND ${CLANG_TIDY_EXECUTABLE}
        -p ${CMAKE_BINARY_DIR}
        ${cpp_sources}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Running clang-tidy static analysis..."
      VERBATIM
    )

    add_custom_target(lint_fix
      COMMAND ${CLANG_TIDY_EXECUTABLE}
        -p ${CMAKE_BINARY_DIR}
        --fix --fix-errors
        ${cpp_sources}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Fixing clang-tidy issues..."
      VERBATIM
    )
  endif()

  # ==========================================================================
  # Test Runner Target
  # ==========================================================================
  add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND}
      --test-dir ${CMAKE_BINARY_DIR}
      --output-on-failure
    COMMENT "Running all tests..."
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endif()

ament_package()
