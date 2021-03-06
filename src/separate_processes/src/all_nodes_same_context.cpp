#include <iostream>
#include <thread>

#include "node_definitions.hpp"
#include "eval_args.hpp"

int main(int argc, char* argv[]) {
    EvalArgs args(argc, argv);
    args.print();

    rclcpp::init(argc, argv);

    auto startNode = createNode<StartNode>(args);
    auto interNode = createNode<IntermediateNode>(args);
    auto endNode = createNode<EndNode>(args);

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