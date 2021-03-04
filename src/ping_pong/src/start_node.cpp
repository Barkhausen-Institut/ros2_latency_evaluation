#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;
class StartNode : public rclcpp::Node {
    public:
        StartNode() : Node("start_node") {
            publisher_ = this->create_publisher<std_msgs::msg::Int64>("/start_pub_topic", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&StartNode::timer_callback, this));
            subscription_ = this->create_subscription<std_msgs::msg::Int64>(
                "/start_sub_topic", 10, std::bind(&StartNode::onPong, this, std::placeholders::_1)
            );
        }
    
    private:
        void timer_callback() {
            auto msg = std_msgs::msg::Int64();
            msg.data = 1;
            RCLCPP_INFO(this->get_logger(), "Publishing %s", std::to_string(msg.data).c_str());
            publisher_->publish(msg);
        }

        void onPong(const std_msgs::msg::Int64::SharedPtr msg) const {
            RCLCPP_INFO(this->get_logger(), "I heard %s", std::to_string(msg->data).c_str());
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartNode>());
    rclcpp::shutdown();
    return 0;
}
