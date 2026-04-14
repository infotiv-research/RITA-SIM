cmake_minimum_required(VERSION 3.8)
project(curobo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Ik.srv"
  "srv/IkBatch.srv"
  "srv/Fk.srv"
  "srv/Collisions.srv"
  "srv/AddObject.srv"
  "srv/RemoveObject.srv"
  "msg/ObjectParameters.msg"
  "srv/GenerateRM.srv"
  "action/DataGeneration.action"
  "action/SendTrajectory.action"
  "action/MpcMove.action"
  "srv/MpcStep.srv"
  "srv/MpcReset.srv"
  "srv/TrajectoryGeneration.srv"
  "srv/GetVoxelGrid.srv"
  "srv/GetCollisionDistance.srv"
  "srv/SceneGenerator.srv"
  "srv/SetPlanner.srv"
  "srv/SetLinkCollision.srv"
  "srv/GetPlanners.srv"
  "srv/SetCollisionCache.srv"
  "srv/SetRobotStrategy.srv"
  "srv/GetRobotStrategies.srv"
  "srv/GraspObject.srv"
  "srv/AttachObject.srv"
  "srv/UpdateObjectPoses.srv"
  DEPENDENCIES geometry_msgs nav2_msgs sensor_msgs# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg

)

ament_package()
