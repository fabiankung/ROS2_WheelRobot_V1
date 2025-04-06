cmake_minimum_required(VERSION 3.8)
project(ros2_test2_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

# Create executable file for project.
add_executable( my_node2 src/my_node2.cpp)  # Create executable or binary file from.
ament_target_dependencies(my_node2 rclcpp example_interfaces) # List down all the libraries needed.

# Set installation of executable
# Note - After we build the package, we need to tell CMAKE where to install the executable files so
# that we can run these with ROS2 command later.
install(TARGETS
  my_node2
  #DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()