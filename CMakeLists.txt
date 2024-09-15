cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(hesai_ros_driver)

#=======================================
# Version information
#=======================================
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  5)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

#========================
# Project details / setup
#========================
set(PROJECT_NAME hesai_ros_driver)

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

add_definitions(-std=c++17)
add_compile_options(-Wall)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(backward_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(rosidl_typesupport_c REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UdpPacket.msg"
  "msg/UdpFrame.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

generate_parameter_library(params
  src/params.yaml
)


#Driver core
add_subdirectory(3rdparty/HesaiLidar_SDK_2.0)

#========================
# Build Setup
#========================

add_executable(hesai_ros_driver_node
  src/hesai_ros_driver_node.cpp
  src/source_driver.cpp
)

target_include_directories(hesai_ros_driver_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  3rdparty/HesaiLidar_SDK_2.0/
  3rdparty/HesaiLidar_SDK_2.0/libhesai
  3rdparty/HesaiLidar_SDK_2.0/libhesai/Lidar
  3rdparty/HesaiLidar_SDK_2.0/libhesai/UdpParser
  3rdparty/HesaiLidar_SDK_2.0/libhesai/UdpParser/include
  3rdparty/HesaiLidar_SDK_2.0/libhesai/UdpParser/src
  3rdparty/HesaiLidar_SDK_2.0/libhesai/UdpProtocol
  3rdparty/HesaiLidar_SDK_2.0/libhesai/Source/include
  3rdparty/HesaiLidar_SDK_2.0/libhesai/Container/include
  3rdparty/HesaiLidar_SDK_2.0/libhesai/Container/src
  3rdparty/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu
  3rdparty/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu/include
  3rdparty/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu/src
  3rdparty/HesaiLidar_SDK_2.0/libhesai/PtcClient/include
  3rdparty/HesaiLidar_SDK_2.0/libhesai/PtcParser/include
  3rdparty/HesaiLidar_SDK_2.0/libhesai/PtcParser
  3rdparty/HesaiLidar_SDK_2.0/libhesai/Logger/include
  3rdparty/HesaiLidar_SDK_2.0/libhesai/include
  3rdparty/HesaiLidar_SDK_2.0/driver
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(hesai_ros_driver_node                   
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
  params
  source_lib
  container_lib
  ptcClient_lib
  ptcParser_lib
  log_lib
)   

ament_target_dependencies(hesai_ros_driver_node 
  "ament_index_cpp"
  "rcl_interfaces"
  "rclcpp" 
  "rcutils"
  "std_msgs" 
  "sensor_msgs" 
)
rosidl_target_interfaces(hesai_ros_driver_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  hesai_ros_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  source_lib
  container_lib
  ptcClient_lib
  ptcParser_lib
  platutils_lib
  log_lib
  DESTINATION lib/)

install(DIRECTORY
  correction
  launch
  params
  rviz
  DESTINATION share/${PROJECT_NAME})
  

ament_package()
