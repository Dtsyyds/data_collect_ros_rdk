cmake_minimum_required(VERSION 3.10)
project(eddy_driver)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(inspection_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/EddyState.msg" DEPENDENCIES builtin_interfaces)

add_library(eddy_driver_lib STATIC src/eddy_controller.cpp)
target_include_directories(eddy_driver_lib PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/eddy_driver")
target_link_libraries(eddy_driver_lib pthread)

add_executable(eddy_driver_node src/eddy_driver_node.cpp src/eddy_driver_main.cpp)
ament_target_dependencies(eddy_driver_node diagnostic_msgs inspection_interfaces rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(eddy_driver_node eddy_driver_lib ${cpp_typesupport_target})
target_include_directories(eddy_driver_node PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/eddy_driver"
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")

install(TARGETS eddy_driver_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS eddy_driver_lib ARCHIVE DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  diagnostic_msgs inspection_interfaces rosidl_default_runtime rclcpp std_msgs)
ament_package()
