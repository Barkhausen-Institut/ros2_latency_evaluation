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

        void onPong(const ping_pong_interfaces::msg::PingPong::SharedPtr msg) {
            auto pong_received_timestamp = get_timestamp();
            uint64_t msg_delay = pong_received_timestamp - msg->ping_timestamp;
            delays_in_ms_.push_back(msg_delay/1000);

            double meanSamples = mean(delays_in_ms_);
            double varianceSamples = variance(delays_in_ms_);

            stats_.push_back(std::make_tuple(meanSamples, varianceSamples));
    
            RCLCPP_INFO(this->get_logger(), "Updated mean of latency in ms: %s", 
                std::to_string(meanSamples).c_str());
            RCLCPP_INFO(this->get_logger(), "Updated Variance of latency in ms: %s", 
                std::to_string(varianceSamples).c_str());
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ping_pong_interfaces::msg::PingPong>::SharedPtr publisher_;
        rclcpp::Subscription<ping_pong_interfaces::msg::PingPong>::SharedPtr subscription_;

        std::vector<uint64_t> delays_in_ms_;
        std::vector<std::tuple<double, double>> stats_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartNode>());
    rclcpp::shutdown();
    return 0;
}
