#include <chrono>
#include <string>
#include "utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ping_pong_interfaces/msg/ping_pong.hpp"

using namespace std::chrono_literals;
class StartNode : public rclcpp::Node {
    public:
        StartNode() : Node("start_node") {
            publisher_ = this->create_publisher<ping_pong_interfaces::msg::PingPong>("/start_pub_topic", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&StartNode::timer_callback, this));
            subscription_ = this->create_subscription<ping_pong_interfaces::msg::PingPong>(
                "/start_sub_topic", 10, std::bind(&StartNode::onPong, this, std::placeholders::_1)
            );
        }
    
    private:
        void timer_callback() {
            auto msg = ping_pong_interfaces::msg::PingPong();
            auto now = get_timestamp();
            msg.ping_timestamp = now;
            publisher_->publish(msg);
        }

        void onPong(const ping_pong_interfaces::msg::PingPong::SharedPtr msg) const {
            RCLCPP_INFO(this->get_logger(), "I heard %s", std::to_string(msg->pong_timestamp).c_str());
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ping_pong_interfaces::msg::PingPong>::SharedPtr publisher_;
        rclcpp::Subscription<ping_pong_interfaces::msg::PingPong>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartNode>());
    rclcpp::shutdown();
    return 0;
}
