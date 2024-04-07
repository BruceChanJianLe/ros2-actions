cmake_minimum_required(VERSION 3.8)
project(move_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(dependencies_pkgs
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  geometry_msgs
  move_base_msgs
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

# BUILD
set(LIBRARY_NAME "move_base_action")
# create action server
add_library(${LIBRARY_NAME}_server
  ./src/server/move_base_action_server.cpp
)
ament_target_dependencies(${LIBRARY_NAME}_server ${dependencies_pkgs})
target_include_directories(${LIBRARY_NAME}_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(${LIBRARY_NAME}_server
  PLUGIN "mb::MoveBaseActionServer"
  EXECUTABLE ${LIBRARY_NAME}_server_component_node
)

# create action client
add_library(${LIBRARY_NAME}_client
  ./src/client/move_base_action_client.cpp
)
ament_target_dependencies(${LIBRARY_NAME}_client ${dependencies_pkgs})
target_include_directories(${LIBRARY_NAME}_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(${LIBRARY_NAME}_client
  PLUGIN "mb::MoveBaseActionClient"
  EXECUTABLE ${LIBRARY_NAME}_client_component_node
)

# create action server node
add_executable(${LIBRARY_NAME}_server_node
  ./src/server/move_base_action_server_node.cpp
)
ament_target_dependencies(${LIBRARY_NAME}_server_node ${dependencies_pkgs})
target_include_directories(${LIBRARY_NAME}_server_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIBRARY_NAME}_server_node ${LIBRARY_NAME}_server)

# create action client node
add_executable(${LIBRARY_NAME}_client_node
  ./src/client/move_base_action_client_node.cpp
)
ament_target_dependencies(${LIBRARY_NAME}_client_node ${dependencies_pkgs})
target_include_directories(${LIBRARY_NAME}_client_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIBRARY_NAME}_client_node ${LIBRARY_NAME}_client)

# INSTALL
# install libraries
install(TARGETS ${LIBRARY_NAME}_server ${LIBRARY_NAME}_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# install nodes
install(TARGETS ${LIBRARY_NAME}_server_node ${LIBRARY_NAME}_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

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

ament_export_libraries(${LIBRARY_NAME}_server ${LIBRARY_NAME}_client)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies_pkgs})

ament_package()
