#include <chrono>
#include <string>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "ping_pong_interfaces/msg/ping_pong.hpp"

#include "utils.hpp"

using namespace std::chrono_literals;
class StartNode : public rclcpp::Node {
    public:
        StartNode() : Node("start_node") {
            publisher_ = this->create_publisher<ping_pong_interfaces::msg::PingPong>("/start_pub_topic", 10);
            timer_ = this->create_wall_timer(
                100ms, std::bind(&StartNode::timer_callback, this));
        }
    
    private:
        void timer_callback() {
            auto msg = ping_pong_interfaces::msg::PingPong();
            auto now = get_timestamp();
            msg.ping_timestamp = now;
            publisher_->publish(msg);
	    RCLCPP_INFO(this->get_logger(), "Published msg");
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ping_pong_interfaces::msg::PingPong>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartNode>());
    rclcpp::shutdown();
    return 0;
}
