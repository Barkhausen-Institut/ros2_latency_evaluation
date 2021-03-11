#include "node_definitions.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntermediateNode>("test"));
    rclcpp::shutdown();
    return 0;
}
