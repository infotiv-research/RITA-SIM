cmake_minimum_required(VERSION 3.22)
project(ur_robotiq_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY srdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/isaac_environment_collision_publisher.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/cumotion_planner_upstream_framefix.py DESTINATION lib/${PROJECT_NAME})
