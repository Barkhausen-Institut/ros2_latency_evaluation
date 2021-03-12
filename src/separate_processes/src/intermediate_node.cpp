#include "node_definitions.hpp"
#include "eval_args.hpp"

int main(int argc, char* argv[]) {
    EvalArgs args(argc, argv);

    rclcpp::init(argc, argv);
    auto node = createNode<IntermediateNode>(args);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
