cmake_minimum_required(VERSION 3.8)
project(pb2025_chassis_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rmoss_base REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/chassis_bridge_node.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rmoss_base
  rmoss_interfaces
  geometry_msgs)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "pb2025_chassis_bridge::ChassisBridgeNode"
  EXECUTABLE chassis_bridge_node)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()
