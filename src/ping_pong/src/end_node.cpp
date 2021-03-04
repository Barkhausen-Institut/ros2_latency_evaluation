#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "utils.hpp"

using namespace std::chrono_literals;
class EndNode : public rclcpp::Node {
    public:
        EndNode() : Node("end_node") {
            publisher_ = this->create_publisher<std_msgs::msg::UInt64>("/start_sub_topic", 10);
            subscription_ = this->create_subscription<std_msgs::msg::UInt64>(
                "/start_pub_topic", 10, std::bind(&EndNode::onPing, this, std::placeholders::_1)
            );
        }
    

    private:
        void onPing(const std_msgs::msg::UInt64::SharedPtr msg) const {
            auto pubMsg = std_msgs::msg::UInt64();
            auto now = get_timestamp();
            pubMsg.data = now;
            publisher_->publish(pubMsg);
            RCLCPP_INFO(this->get_logger(), "Publishing %s", std::to_string(pubMsg.data).c_str());
        }
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndNode>());
    rclcpp::shutdown();
    return 0;
}
