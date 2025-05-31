cmake_minimum_required(VERSION 3.10)
project(end_sphere_pose_listener)

# Required for ROS 2 packages
find_package(ament_cmake REQUIRED)

# Find Gazebo Transport
find_package(gz-transport12 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

# Define executable
add_executable(end_sphere_listener EndSpherePoseListener.cc)

# Link to gz-transport
ament_target_dependencies(end_sphere_listener
  rclcpp
  px4_msgs
  geometry_msgs
  gz-transport12
)

target_link_libraries(end_sphere_listener Eigen3::Eigen)


# Install binary
install(TARGETS end_sphere_listener
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
