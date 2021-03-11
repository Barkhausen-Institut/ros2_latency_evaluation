#include "node_definitions.hpp"
#include "eval_args.hpp"

int main(int argc, char* argv[]) {
    EvalArgs args(argc, argv);
    auto node = createNode<IntermediateNode>(args);
    rclcpp::init(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
