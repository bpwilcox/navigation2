cmake_minimum_required(VERSION 3.5)
project(nav2_parameter_server)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_util REQUIRED)


nav2_package()

include_directories(
  include
)

set(executable_name parameter_server)
set(library_name parameter_library)

set(dependencies
  rclcpp
  nav2_util
  rclcpp_components
)

add_library(${library_name} SHARED
  src/parameter_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

#add_executable(${executable_name}
#  src/main.cpp
#)

rclcpp_components_register_node(${library_name}
  PLUGIN "nav2_parameter_server::ParameterServer"
  EXECUTABLE ${executable_name})

#ament_target_dependencies(${executable_name}
#  ${dependencies}
#)

#target_link_libraries(${executable_name} ${library_name})

#install(TARGETS ${executable_name} ${library_name}

install(TARGETS parameter_library
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
