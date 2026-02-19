#!/bin/bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
USER_WS="${USER_WS:-/ros2_ws}"
FORCE_REBUILD="${CUMOTION_FORCE_REBUILD:-false}"

required_system_packages=(
  isaac_ros_cumotion
  isaac_ros_cumotion_moveit
  moveit_ros_move_group
  joint_state_publisher
  rviz2
)

required_workspace_packages=(
  ur_description
  robotiq_description
  ur_robotiq_description
  ur_robotiq_moveit_config
  ur_robotiq_servo
)

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found."
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${USER_WS}/install/setup.bash" ]; then
  source "${USER_WS}/install/setup.bash"
fi
set -u

missing_system_packages=()
for pkg in "${required_system_packages[@]}"; do
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    missing_system_packages+=("${pkg}")
  fi
done

if [ "${#missing_system_packages[@]}" -gt 0 ]; then
  echo "ERROR: Missing ROS packages in cumotion image:"
  printf '  - %s\n' "${missing_system_packages[@]}"
  echo "Rebuild the cumotion image: docker compose -f setup/docker-compose.ros2.yaml build cumotion"
  exit 1
fi

if ! python3 -c "import torch; import warp; import curobo; from isaac_ros_cumotion.cumotion_planner import CumotionActionServer" >/dev/null 2>&1; then
  echo "ERROR: cuMotion Python runtime import check failed (torch/warp/curobo/isaac_ros_cumotion)."
  echo "Rebuild the cumotion image: docker compose -f setup/docker-compose.ros2.yaml build cumotion"
  exit 1
fi

workspace_pkg_set="$(colcon list --base-paths "${USER_WS}/src" --names-only 2>/dev/null || true)"
missing_from_workspace=()
for pkg in "${required_workspace_packages[@]}"; do
  if ! grep -qx "${pkg}" <<<"${workspace_pkg_set}"; then
    missing_from_workspace+=("${pkg}")
  fi
done

if [ "${#missing_from_workspace[@]}" -gt 0 ]; then
  echo "ERROR: Required workspace packages are missing in ${USER_WS}/src:"
  printf '  - %s\n' "${missing_from_workspace[@]}"
  exit 1
fi

packages_to_build=()
if [ "${FORCE_REBUILD}" = "true" ]; then
  packages_to_build=("${required_workspace_packages[@]}")
else
  for pkg in "${required_workspace_packages[@]}"; do
    if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
      packages_to_build+=("${pkg}")
    fi
  done
fi

if [ "${#packages_to_build[@]}" -gt 0 ]; then
  echo "Bootstrapping workspace packages for cumotion:"
  printf '  - %s\n' "${packages_to_build[@]}"
  (
    cd "${USER_WS}"
    colcon build --symlink-install --packages-select "${packages_to_build[@]}"
  )
fi

if [ -f "${USER_WS}/install/setup.bash" ]; then
  set +u
  source "${USER_WS}/install/setup.bash"
  set -u
fi

missing_after_build=()
for pkg in "${required_workspace_packages[@]}"; do
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    missing_after_build+=("${pkg}")
  fi
done

if [ "${#missing_after_build[@]}" -gt 0 ]; then
  echo "ERROR: Workspace bootstrap incomplete, missing packages after build:"
  printf '  - %s\n' "${missing_after_build[@]}"
  exit 1
fi
