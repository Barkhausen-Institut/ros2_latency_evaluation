#include "node_definitions.hpp"
#include "eval_args.hpp"

int main(int argc, char* argv[]) {
    EvalArgs args(argc, argv);
    args.print();

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node;
    if (args.nodeIndex == 0) {
      std::cout << "Creating a START node" << std::endl;
      node = createNode<StartNode>(args);
    }
    else if (args.nodeIndex == args.noNodes - 1) {
      std::cout << "Creating an END node" << std::endl;
      node = createNode<EndNode>(args);
    }
    else {
      std::cout << "Creating INTERMEDIATE Node Nr. " << args.nodeIndex + 1 << std::endl;
      node = createNode<IntermediateNode>(args);
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
