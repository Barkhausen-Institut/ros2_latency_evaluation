#include "node_definitions.hpp"
#include "eval_args.hpp"

int main(int argc, char* argv[]) {

    EvalArgs args(argc, argv);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartNode>(args));
    rclcpp::shutdown();
    return 0;
}