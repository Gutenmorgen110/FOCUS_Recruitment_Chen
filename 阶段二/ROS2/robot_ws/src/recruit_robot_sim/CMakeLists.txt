cmake_minimum_required(VERSION 3.8)
project(recruit_robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories into the share space
install(
  DIRECTORY urdf worlds launch rviz meshes config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
