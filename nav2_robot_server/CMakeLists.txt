cmake_minimum_required(VERSION 3.5)
project(nav2_robot_server)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name robot_server)
set(library_name nav2_robot_server)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  urdf
  geometry_msgs
  nav_msgs
  nav2_msgs
  nav2_util
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_library(${library_name} SHARED
  src/robot_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name}
  src/main.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

add_executable(robot_param_client
  src/robot_param_client.cpp
)

ament_target_dependencies(robot_param_client
  ${dependencies}
)

install(TARGETS ${executable_name} ${library_name} robot_param_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  #add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(urdf)
ament_export_dependencies(nav_msgs)

ament_package()
