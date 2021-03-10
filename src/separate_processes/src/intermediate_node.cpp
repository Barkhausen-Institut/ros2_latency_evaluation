#include "node_definitions.hpp"

int main(int argc, char* argv[]) {
    auto node = createNode<IntermediateNode>("100b");
    rclcpp::init(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
