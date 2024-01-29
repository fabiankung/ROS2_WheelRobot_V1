cmake_minimum_required(VERSION 3.8)
project(wheelrobotnav2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Ask the compiler to make a copy of the urdf folder 
# in the path in Destination.
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
