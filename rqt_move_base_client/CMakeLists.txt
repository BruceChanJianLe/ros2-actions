cmake_minimum_required(VERSION 3.5)
project(rqt_move_base_client)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

set(dependencies_pkgs
  rclcpp
  move_base
  rqt_gui_cpp
  Qt5Widgets
  qt_gui_cpp
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

# Start
# Prepare to create rqt plugin as cpp library

# Define source files
set(${PROJECT_NAME}_SRCS
  src/rqt_move_base_client.cpp
)

# Define header files
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/rqt_move_base_client.hpp
)

# Define ui file
set(${PROJECT_NAME}_UIS
  resource/rqt_move_base_client.ui
)

# Define include directories
set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${rclcpp_INCLUDE_DIRS}
  ${move_base_INCLUDE_DIRS}
  ${qt_gui_cpp_INCLUDE_DIRS}
  ${rqt_gui_cpp_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Qt wrapper
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
)

add_library(rqt_move_base_client
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_MOCS}
  ${${PROJECT_NAME}_UIS_H}
)
ament_target_dependencies(rqt_move_base_client
  rclcpp
  move_base
  qt_gui_cpp
  rqt_gui_cpp
)
target_link_libraries(rqt_move_base_client
  Qt5::Widgets
)
# Ends

install(
  TARGETS rqt_move_base_client
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(PROGRAMS scripts/rqt_move_base_client
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

# Install for ros2 launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")
ament_export_libraries(rqt_move_base_client)

ament_package()
