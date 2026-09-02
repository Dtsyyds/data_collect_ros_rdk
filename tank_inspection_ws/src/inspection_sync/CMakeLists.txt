cmake_minimum_required(VERSION 3.8)
project(inspection_sync)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(inspection_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(inspection_sync_node src/inspection_sync_node.cpp)
target_compile_features(inspection_sync_node PUBLIC cxx_std_17)
target_include_directories(inspection_sync_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(inspection_sync_node
  diagnostic_msgs geometry_msgs inspection_interfaces nav_msgs rclcpp sensor_msgs)

install(TARGETS inspection_sync_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sync_core test/test_sync_core.cpp)
  if(TARGET test_sync_core)
    target_compile_features(test_sync_core PUBLIC cxx_std_17)
    target_include_directories(test_sync_core PRIVATE include)
    ament_target_dependencies(test_sync_core geometry_msgs)
  endif()
endif()

ament_package()
