cmake_minimum_required(VERSION 3.8)
project(sensor_gui)

# Enable strict warnings for C++ if applicable (optional for Python packages)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install the Python script
install(
  PROGRAMS
  scripts/gui_node.py  # Your Python script filename
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Ament package definition
ament_package()

