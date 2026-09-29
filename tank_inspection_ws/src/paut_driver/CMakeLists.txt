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
# v0(旧线格式, :12345): 代码冻结保留 —— bags/paut_real 等 5 个 bag 依赖其消息类型。
add_library(paut_udp_receiver STATIC src/paut_udp_receiver.cpp)
target_include_directories(paut_udp_receiver PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/paut_driver")
target_link_libraries(paut_udp_receiver pthread)

# v1(新线格式, :12346) 协议层 + 接收器静态库。
# 协议层(paut_v1_protocol)是纯函数, 不依赖 socket/ROS, 可单独做单元测试。
add_library(paut_v1_receiver STATIC
  src/paut_v1_protocol.cpp
  src/paut_v1_receiver.cpp)
target_include_directories(paut_v1_receiver PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/paut_driver")
target_link_libraries(paut_v1_receiver pthread)

add_executable(paut_driver_node src/paut_driver_node.cpp src/paut_driver_main.cpp)
ament_target_dependencies(paut_driver_node diagnostic_msgs inspection_interfaces rclcpp std_msgs)
# PautFrame 来自 inspection_interfaces 包, 无需 rosidl_get_typesupport_target。
target_link_libraries(paut_driver_node paut_v1_receiver)
target_include_directories(paut_driver_node PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/paut_driver")

install(TARGETS paut_driver_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS paut_udp_receiver ARCHIVE DESTINATION lib/${PROJECT_NAME})
install(TARGETS paut_v1_receiver ARCHIVE DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  # v1 协议解析单元测试。参考字节由 Python 合成源(scripts/send_paut_12346_synth.py)
  # 生成后内嵌, 与 C++ 解析器构成**跨实现比对** —— 能抓出偏移写错这类静默错误。
  # 只用到纯协议层, 不涉及 ROS 节点/设备/socket。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_v1_protocol test/test_v1_protocol.cpp)
  if(TARGET test_v1_protocol)
    target_compile_features(test_v1_protocol PUBLIC cxx_std_17)
    target_link_libraries(test_v1_protocol paut_v1_receiver)
  endif()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(diagnostic_msgs inspection_interfaces rclcpp std_msgs)
ament_package()
