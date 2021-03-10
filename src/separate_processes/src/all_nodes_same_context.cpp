#include "node_definitions.hpp"
#include <iostream>

#include <thread>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto startNode = createNode<StartNode>("100b");
    auto interNode = createNode<IntermediateNode>("100b");
    auto endNode = createNode<EndNode>("100b");

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