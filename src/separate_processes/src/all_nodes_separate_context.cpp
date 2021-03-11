#include <iostream>
#include <thread>

#include "node_definitions.hpp"
#include "eval_args.hpp"

int main(int argc, char* argv[]) {
    EvalArgs args(argc, argv);
    args.print();
    rclcpp::init(argc, argv);

    auto contextStartNode = std::make_shared<rclcpp::Context>();
    contextStartNode->init(argc, argv);

    auto contextInterNode = std::make_shared<rclcpp::Context>();
    contextInterNode->init(argc, argv);

    auto contextEndNode = std::make_shared<rclcpp::Context>();
    contextEndNode->init(argc, argv);

    rclcpp::NodeOptions optStartNode = rclcpp::NodeOptions();
    optStartNode.context(contextStartNode);
    rclcpp::NodeOptions optInterNode = rclcpp::NodeOptions();
    optInterNode.context(contextInterNode);
    rclcpp::NodeOptions optEndNode = rclcpp::NodeOptions();
    optEndNode.context(contextEndNode);

    auto startNode = std::make_shared<StartNode>(args, optStartNode);
    auto interNode = std::make_shared<IntermediateNode>(optInterNode);
    auto endNode = std::make_shared<EndNode>(optEndNode);

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