cmake_minimum_required(VERSION 3.8)
project(URDFF_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts directly
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  urdf
  worlds
  meshes
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
