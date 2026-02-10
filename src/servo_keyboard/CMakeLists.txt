cmake_minimum_required(VERSION 3.8)
project(servo_keyboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)


add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(servo_keyboard_input PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Include directories
target_include_directories(servo_keyboard_input PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${controller_manager_msgs_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}  # Add this line to include YAML-cpp headers
)

# Link the executable with required libraries
ament_target_dependencies(servo_keyboard_input
  rclcpp
  geometry_msgs
  control_msgs
  controller_manager_msgs
  yaml-cpp  # Add this line to link against YAML-cpp
    ament_index_cpp
)
target_link_libraries(servo_keyboard_input ${YAML_CPP_LIBRARIES})  # Add this line to link against YAML-cpp

# Existing install directive for the executable
install(TARGETS servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME})

# Added install directive for the launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
