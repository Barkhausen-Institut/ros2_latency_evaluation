#include "node_definitions.hpp"

int main(int argc, char* argv[]) {

    Arguments args = parseArgs(argc, argv);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartNode>(args.pubFreq));
    rclcpp::shutdown();
    return 0;
}