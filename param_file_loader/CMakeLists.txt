cmake_minimum_required(VERSION 3.8)
project(param_file_loader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(param_loader_node src/param_loader.cpp)
ament_target_dependencies(param_loader_node rclcpp)

install(TARGETS
  param_loader_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
