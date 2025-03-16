cmake_minimum_required(VERSION 3.8)
project(marl)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Install URDF, Worlds, Scripts, and Launch Files
install(DIRECTORY urdf worlds scripts launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()