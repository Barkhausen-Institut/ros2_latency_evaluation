#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
class FirstNode : public rclcpp::Node {
    public:
        FirstNode() : Node("first_node") {
            publisher_ = this->create_publisher<std_msgs::msg::String>("/first_pub_topic", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&FirstNode::timer_callback, this));
        }
    
    private:
        void timer_callback() {
            auto msg = std_msgs::msg::String();
            msg.data = "hi";
            RCLCPP_INFO(this->get_logger(), "Publishing %s", msg.data.c_str());
            publisher_->publish(msg);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FirstNode>());
    rclcpp::shutdown();
    return 0;
}
