cmake_minimum_required(VERSION 3.8)
project(inspection_livox_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(livox_imu_adapter_node src/livox_imu_adapter_node.cpp)
target_include_directories(livox_imu_adapter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(livox_imu_adapter_node diagnostic_msgs rclcpp sensor_msgs)

install(TARGETS livox_imu_adapter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_conversion test/test_imu_conversion.cpp)
  target_include_directories(test_imu_conversion PRIVATE include)
  ament_target_dependencies(test_imu_conversion sensor_msgs)
endif()

ament_package()
