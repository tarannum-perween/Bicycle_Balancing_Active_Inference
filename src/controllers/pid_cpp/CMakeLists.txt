cmake_minimum_required(VERSION 3.8)
project(pid_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(torque_Pgain_ctl src/torque_Pgain_simple.cpp)
target_compile_features(torque_Pgain_ctl PUBLIC cxx_std_17)
ament_target_dependencies(torque_Pgain_ctl rclcpp std_msgs)

# Install executable targets so ros2 run can find them
install(TARGETS
  torque_Pgain_ctl
  DESTINATION lib/${PROJECT_NAME}
)

# Install setup directories to share folder locations
# This is important for launch files and other resources to be found by ros2 launch
# not sure this is correct...
install(DIRECTORY include launch src config
  DESTINATION share/${PROJECT_NAME}
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
