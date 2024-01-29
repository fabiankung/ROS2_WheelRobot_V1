cmake_minimum_required(VERSION 3.8)
project(robotcore_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(custom_robot_interface REQUIRED)

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

# Create executable file for project.

add_executable(core_node src/corenode.cpp)  # Create executable or binary file.
#ament_target_dependencies(core_node rclcpp example_interfaces) # Set link library
ament_target_dependencies(core_node rclcpp custom_robot_interface) # Set link library

# Set installation of executable.
# Note - after we build the program above, we need to tell CMAKE where to
# install the executable files so that we can run these with ros2 command
# later.
install(TARGETS
  core_node
  #DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
