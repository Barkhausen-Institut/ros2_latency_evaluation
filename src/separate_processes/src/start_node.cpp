#include "node_definitions.hpp"

int main(int argc, char* argv[]) {

    int pubFreq = 1;

    if (argc > 1) {
        pubFreq = atof(argv[1]);
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartNode>(pubFreq));
    rclcpp::shutdown();
    return 0;
}