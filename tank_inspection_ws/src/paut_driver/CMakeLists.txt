cmake_minimum_required(VERSION 3.10)
project(paut_driver)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(inspection_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# UDP 接收/解码器静态库(镜像 eddy_driver 的 eddy_driver_lib)。
add_library(paut_udp_receiver STATIC src/paut_udp_receiver.cpp)
target_include_directories(paut_udp_receiver PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/paut_driver")
target_link_libraries(paut_udp_receiver pthread)

add_executable(paut_driver_node src/paut_driver_node.cpp src/paut_driver_main.cpp)
ament_target_dependencies(paut_driver_node diagnostic_msgs inspection_interfaces rclcpp std_msgs)
# PautFrame 来自 inspection_interfaces 包, 无需 rosidl_get_typesupport_target。
target_link_libraries(paut_driver_node paut_udp_receiver)
target_include_directories(paut_driver_node PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/paut_driver")

install(TARGETS paut_driver_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS paut_udp_receiver ARCHIVE DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(diagnostic_msgs inspection_interfaces rclcpp std_msgs)
ament_package()
