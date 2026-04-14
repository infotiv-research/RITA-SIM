cmake_minimum_required(VERSION 3.14)
project(curobo_hybrid_planning_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(curobo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_hybrid_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/curobo_global_planner.cpp
  src/curobo_mpc_local_solver.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}
  curobo_msgs
  geometry_msgs
  moveit_core
  moveit_hybrid_planning
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

pluginlib_export_plugin_description_file(moveit_hybrid_planning curobo_hybrid_planning_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES curobo_hybrid_planning_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  curobo_msgs
  geometry_msgs
  moveit_core
  moveit_hybrid_planning
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
