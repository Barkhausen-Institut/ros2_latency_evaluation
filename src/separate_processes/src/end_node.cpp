#include <chrono>
#include <ctime>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "ping_pong_interfaces/msg/ping_pong.hpp"


#include "utils.hpp"

using namespace std::chrono_literals;
class EndNode : public rclcpp::Node {
    public:
        EndNode() : Node("end_node") {
            subscription_ = this->create_subscription<ping_pong_interfaces::msg::PingPong>(
                "/end_sub_topic", 10, std::bind(&EndNode::onPong, this, std::placeholders::_1)
            );
	    noMsgs_ = 0;
        }
    

    private:
        void onPong(const ping_pong_interfaces::msg::PingPong::SharedPtr msg) {
    	    noMsgs_++;
	    if (noMsgs_ > 30) {
                auto pong_received_timestamp = get_timestamp();
	        uint64_t msg_delay = pong_received_timestamp - msg->ping_timestamp;
            	delays_in_ms_.push_back(msg_delay/1000);

	        double meanSamples = mean(delays_in_ms_);
                double varianceSamples = variance(delays_in_ms_);

                stats_.push_back(std::make_tuple(meanSamples, varianceSamples));
	        RCLCPP_INFO(this->get_logger(), "latency in ms: %s", std::to_string(msg_delay/1000).c_str());
                RCLCPP_INFO(this->get_logger(), "Updated mean of latency in ms: %s", 
                      std::to_string(meanSamples).c_str());
                RCLCPP_INFO(this->get_logger(), "Updated Variance of latency in ms: %s", 
                      std::to_string(varianceSamples).c_str());
                RCLCPP_INFO(this->get_logger(), "Msgs: %s", std::to_string(noMsgs_).c_str());
	   }
        }
        rclcpp::Subscription<ping_pong_interfaces::msg::PingPong>::SharedPtr subscription_;

        std::vector<uint64_t> delays_in_ms_;
        std::vector<std::tuple<double, double>> stats_;
	int noMsgs_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndNode>());
    rclcpp::shutdown();
    return 0;
}
