#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;
class EndNode : public rclcpp::Node {
    public:
        EndNode() : Node("end_node") {
            publisher_ = this->create_publisher<std_msgs::msg::Int64>("/first_sub_topic", 10);
            subscription_ = this->create_subscription<std_msgs::msg::Int64>(
                "/first_pub_topic", 10, std::bind(&EndNode::onPing, this, std::placeholders::_1)
            );
        }
    
    private:
        void onPing(const std_msgs::msg::Int64::SharedPtr msg) const {
            publisher_->publish(*msg);
        }
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndNode>());
    rclcpp::shutdown();
    return 0;
}
