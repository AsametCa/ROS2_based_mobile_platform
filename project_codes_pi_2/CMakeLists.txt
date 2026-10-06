# ==============================================================================
# @file CMakeLists.txt
# @brief Build configuration for project_codes_pi_2 package.
# ==============================================================================

cmake_minimum_required(VERSION 3.8)
project(project_codes_pi_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# ------------------------------------------------------------------------------
# Executable: Odometry Node (Calculates dead reckoning from encoder velocities)
# ------------------------------------------------------------------------------
add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node rclcpp std_msgs nav_msgs tf2_ros tf2)

# ------------------------------------------------------------------------------
# Executable: Trajectory Publisher (Path comparison and drift tracking)
# ------------------------------------------------------------------------------
add_executable(trajectory_publisher src/trajectory_publisher.cpp)
ament_target_dependencies(trajectory_publisher rclcpp tf2_ros tf2 nav_msgs geometry_msgs)

# ------------------------------------------------------------------------------
# Install Targets & Directories
# ------------------------------------------------------------------------------
install(TARGETS
  odometry_node
  trajectory_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
