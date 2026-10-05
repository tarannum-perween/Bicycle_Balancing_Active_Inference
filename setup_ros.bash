# Source this file from Bash to use ROS Jazzy and this workspace.
_stabletech_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
_stabletech_deps="${_stabletech_root}/.ros-deps/opt/ros/jazzy"
export AMENT_PREFIX_PATH="${_stabletech_deps}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export CMAKE_PREFIX_PATH="${_stabletech_deps}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PATH="${_stabletech_deps}/bin:${PATH}"
export PYTHONPATH="${_stabletech_deps}/lib/python3.12/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
if [[ -f "${_stabletech_root}/install/local_setup.bash" ]]; then
    source "${_stabletech_root}/install/local_setup.bash"
fi
unset _stabletech_root _stabletech_deps
