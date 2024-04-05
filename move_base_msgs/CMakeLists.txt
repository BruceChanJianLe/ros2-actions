cmake_minimum_required(VERSION 3.8)
project(move_base_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)

set(dependencies_pkgs
  rclcpp
  builtin_interfaces
  rosidl_default_generators
  geometry_msgs
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

# BUILD
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveBase.action"
  DEPENDENCIES ${dependencies_pkgs}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
