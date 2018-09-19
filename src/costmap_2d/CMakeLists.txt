cmake_minimum_required(VERSION 3.5)
project(costmap_2d)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(xmlrpcpp2 REQUIRED)

#find_package(dynamic_reconfigure REQUIRED) 
#find_package(laser_geometry REQUIRED)
#find_package(message_filters REQUIRED)
#find_package(message_generation REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(pcl_ros REQUIRED)
#find_package(tf2 REQUIRED)
#find_package(voxel_grid REQUIRED)
find_package(pluginlib REQUIRED)


#find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
	include
	${ament_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	#${PCL_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${xmlrpcpp_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoxelGrid.msg"

	DEPENDENCIES std_msgs geometry_msgs map_msgs
)


add_library(nav2_costmap_2d
  src/array_parser.cpp
  src/costmap_2d.cpp
  #src/observation_buffer.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_math.cpp
  src/footprint.cpp
  src/costmap_layer.cpp
)

ament_target_dependencies(nav2_costmap_2d geometry_msgs nav_msgs map_msgs tf2_ros tf2 xmlrpcpp2 pluginlib tf2_geometry_msgs)
target_link_libraries(nav2_costmap_2d
  #${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ament_LIBRARIES}
)

#add_library(layers
  #plugins/inflation_layer.cpp
  #plugins/obstacle_layer.cpp
  #plugins/static_layer.cpp
  #plugins/voxel_layer.cpp
  #src/observation_buffer.cpp
#)
#ament_target_dependencies(layers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${ament_EXPORTED_TARGETS})
#target_link_libraries(layers
  #costmap_2d
#)

#add_executable(costmap_2d_markers src/costmap_2d_markers.cpp)
#ament_target_dependencies(costmap_2d_markers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${ament_EXPORTED_TARGETS})
#target_link_libraries(costmap_2d_markers
    #costmap_2d
#)

#add_executable(costmap_2d_cloud src/costmap_2d_cloud.cpp)
#ament_target_dependencies(costmap_2d_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${ament_EXPORTED_TARGETS})
#target_link_libraries(costmap_2d_cloud
    #costmap_2d
#)

#add_executable(costmap_2d_node src/costmap_2d_node.cpp)
#ament_target_dependencies(costmap_2d_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${ament_EXPORTED_TARGETS})
#target_link_libraries(costmap_2d_node
    #costmap_2d
#)


#install( TARGETS
    #costmap_2d_markers
    #costmap_2d_cloud
    #costmap_2d_node
    #DESTINATION lib/${PROJECT_NAME}
#)

install(TARGETS nav2_costmap_2d DESTINATION lib/${PROJECT_NAME}
)

#install(FILES costmap_plugins.xml
  #DESTINATION share/${PROJECT_NAME}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()

