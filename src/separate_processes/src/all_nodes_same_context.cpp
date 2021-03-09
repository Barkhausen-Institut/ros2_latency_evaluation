#include "node_definitions.hpp"
#include <iostream>

#include <thread>

int main(int argc, char* argv[]) {
    float pubFreq = 1.;

    if (argc > 1) {
        pubFreq = atof(argv[1]);
    }
    rclcpp::init(argc, argv);

    auto startNode = std::make_shared<StartNode>(pubFreq);
    auto interNode = std::make_shared<IntermediateNode>();
    auto endNode = std::make_shared<EndNode>();

    auto exStartNode = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
    auto exInterNode = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
    auto exEndNode = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();

    exStartNode->add_node(startNode);
    exInterNode->add_node(interNode);
    exEndNode->add_node(endNode);

    std::thread threadStartNode([=] {exStartNode->spin(); } );
    std::thread threadIntermediateNode([=] {exInterNode->spin(); } );
    std::thread threadEndNode([=] {exEndNode->spin(); } );

    threadStartNode.join();
    threadIntermediateNode.join();
    threadEndNode.join();

    rclcpp::shutdown();
    return 0;
}