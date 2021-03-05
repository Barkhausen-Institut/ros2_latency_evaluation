#include "node_definitions.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartNode>());
    rclcpp::shutdown();
    return 0;
}