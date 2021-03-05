#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ping_pong_interfaces/msg/ping_pong.hpp"

#include "utils.hpp"

using namespace std::chrono_literals;
class IntermediateNode : public rclcpp::Node {
    public:
        IntermediateNode() : Node("intermediate_node") {
            publisher_ = this->create_publisher<ping_pong_interfaces::msg::PingPong>("/end_sub_topic", 10);
            subscription_ = this->create_subscription<ping_pong_interfaces::msg::PingPong>(
                "/start_pub_topic", 10, std::bind(&IntermediateNode::onPing, this, std::placeholders::_1)
            );
        }
    

    private:
        void onPing(const ping_pong_interfaces::msg::PingPong::SharedPtr msg) const {
            publisher_->publish(*msg);
	    RCLCPP_INFO(this->get_logger(), "I received a msg");
        }
        rclcpp::Publisher<ping_pong_interfaces::msg::PingPong>::SharedPtr publisher_;
        rclcpp::Subscription<ping_pong_interfaces::msg::PingPong>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntermediateNode>());
    rclcpp::shutdown();
    return 0;
}
