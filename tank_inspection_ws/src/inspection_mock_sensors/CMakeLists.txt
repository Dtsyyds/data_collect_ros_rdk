cmake_minimum_required(VERSION 3.8)
project(inspection_mock_sensors)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(inspection_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
# nlohmann_json 可能未随系统安装(仅 header)。系统有则用系统 CMake 包,
# 否则回退到仓库 vendored 单头(<nlohmann/json.hpp>), 保证无额外系统依赖也能构建。
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
  set_target_properties(nlohmann_json::nlohmann_json PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
endif()
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mock_sensors_node src/mock_sensors_node.cpp)
target_compile_features(mock_sensors_node PUBLIC cxx_std_17)
target_include_directories(mock_sensors_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mock_sensors_node nlohmann_json::nlohmann_json)
ament_target_dependencies(mock_sensors_node
  geometry_msgs inspection_interfaces nav_msgs rclcpp sensor_msgs tf2_ros)

install(TARGETS mock_sensors_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_trajectory test/test_mission_trajectory.cpp)
  if(TARGET test_mission_trajectory)
    target_compile_features(test_mission_trajectory PUBLIC cxx_std_17)
    target_include_directories(test_mission_trajectory PRIVATE include)
    target_link_libraries(test_mission_trajectory nlohmann_json::nlohmann_json)
  endif()
endif()
ament_package()
