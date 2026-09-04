/**
 * @file paut_driver_main.cpp
 * @brief PAUT 采集驱动节点入口
 */
#include <rclcpp/rclcpp.hpp>
#include "paut_driver/paut_driver_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<paut_driver::PautDriverNode>());
    rclcpp::shutdown();
    return 0;
}
