/**
 * @file eddy_driver_main.cpp
 * @brief Eddy 采集驱动节点入口
 */
#include <rclcpp/rclcpp.hpp>
#include "eddy_driver/eddy_driver_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<eddy_driver::EddyDriverNode>());
    rclcpp::shutdown();
    return 0;
}
