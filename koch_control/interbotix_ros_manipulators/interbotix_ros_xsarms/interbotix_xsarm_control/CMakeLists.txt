cmake_minimum_required(VERSION 3.5)
project(interbotix_xsarm_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (required for MoveIt2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build endeffector_koch executable
add_executable(endeffector_koch endeffector_koch.cpp)
ament_target_dependencies(endeffector_koch 
  rclcpp 
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
)

# Build endeffector_koch_v2 executable (improved version)
add_executable(endeffector_koch_v2 endeffector_koch_v2.cpp)
ament_target_dependencies(endeffector_koch_v2 
  rclcpp 
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
)

# Install the executables
install(TARGETS
  endeffector_koch
  endeffector_koch_v2
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(
  PROGRAMS
    scripts/simple_endeffector_control.py
    scripts/simple_moveit_control.py
    demos/bartender.py
    demos/ee_cartesian_trajectory.py
    demos/ee_pose_components.py
    demos/ee_pose_matrix_control.py
    demos/gripper_control.py
    demos/joint_current_control.py
    demos/joint_position_control.py
    demos/joint_pwm_control.py
    demos/joint_trajectory_control.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
