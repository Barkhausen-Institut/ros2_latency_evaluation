#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "ping_pong_interfaces/msg/ping_pong.hpp"


#include "utils.hpp"

using namespace std::chrono_literals;
class EndNode : public rclcpp::Node {
    public:
        EndNode() : Node("end_node") {
            subscription_ = this->create_subscription<ping_pong_interfaces::msg::PingPong>(
                "/end_sub_topic", 10, std::bind(&EndNode::onPing, this, std::placeholders::_1)
            );
        }
    

    private:
        void onPing(const ping_pong_interfaces::msg::PingPong::SharedPtr msg) const {
            auto now = get_timestamp();
            RCLCPP_INFO(this->get_logger(), "PING: %s", std::to_string(msg->ping_timestamp).c_str());
            RCLCPP_INFO(this->get_logger(), "PONG: %s", std::to_string(msg->pong_timestamp).c_str());
        }
        rclcpp::Subscription<ping_pong_interfaces::msg::PingPong>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndNode>());
    rclcpp::shutdown();
    return 0;
}
