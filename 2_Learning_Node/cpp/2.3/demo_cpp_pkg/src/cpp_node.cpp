#include <iostream>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    std::cout << "参数数量 = " << argc << std::endl;
    std::cout << "参数名称 = " << argv[0] << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("cpp_node");
    RCLCPP_INFO(node->get_logger(), "你好，C++ 节点！");
    RCLCPP_WARN(node->get_logger(), "你好，C++ 节点！");
    RCLCPP_ERROR(node->get_logger(), "你好，C++ 节点！");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}