#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "ping_pong_interfaces/msg/ping_pong.hpp"


#include "utils.hpp"

using namespace std::chrono_literals;
class EndNode : public rclcpp::Node {
    public:
        EndNode() : Node("end_node") {
            publisher_ = this->create_publisher<ping_pong_interfaces::msg::PingPong>("/start_sub_topic", 10);
            subscription_ = this->create_subscription<ping_pong_interfaces::msg::PingPong>(
                "/start_pub_topic", 10, std::bind(&EndNode::onPing, this, std::placeholders::_1)
            );
        }
    

    private:
        void onPing(const ping_pong_interfaces::msg::PingPong::SharedPtr msg) const {
            auto now = get_timestamp();
            msg->pong_timestamp = now;
            publisher_->publish(*msg);
        }
        rclcpp::Publisher<ping_pong_interfaces::msg::PingPong>::SharedPtr publisher_;
        rclcpp::Subscription<ping_pong_interfaces::msg::PingPong>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndNode>());
    rclcpp::shutdown();
    return 0;
}
