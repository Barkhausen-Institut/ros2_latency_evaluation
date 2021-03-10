#include <chrono>
#include <string>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"

#include "ping_pong_interfaces/msg/stamped100b.hpp"
#include "ping_pong_interfaces/msg/stamped1kb.hpp"
#include "ping_pong_interfaces/msg/stamped10kb.hpp"
#include "ping_pong_interfaces/msg/stamped100kb.hpp"
#include "ping_pong_interfaces/msg/stamped500kb.hpp"

#include "utils.hpp"

using namespace std::chrono_literals;

template <class MsgType>
class StartNode : public rclcpp::Node {
    public:
        StartNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("start_node", "", opt) {
            publisher_ = this->create_publisher<MsgType>("/start_pub_topic", 10);
            timer_ = this->create_wall_timer(
                100ms, std::bind(&StartNode::timer_callback, this));
        }
    
    private:
        void timer_callback() {
            auto msg = MsgType();
            auto now = get_timestamp();
            msg.info.timestamp = now;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published msg");
        }
        rclcpp::TimerBase::SharedPtr timer_;
        typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
};

template <class MsgType>
class IntermediateNode : public rclcpp::Node {
    public:
        IntermediateNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("intermediate_node", "", opt) {
            publisher_ = this->create_publisher<MsgType>("/end_sub_topic", 10);
            subscription_ = this->create_subscription<MsgType>(
                "/start_pub_topic", 10, std::bind(&IntermediateNode::onPing, this, std::placeholders::_1)
            );
        }


    private:
        void onPing(const typename MsgType::SharedPtr msg) const {
            publisher_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "I received a msg");
        }
        typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
        typename rclcpp::Subscription<MsgType>::SharedPtr subscription_;
};

template <class MsgType>
class EndNode : public rclcpp::Node {
    public:
        EndNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("end_node", "", opt) {
            subscription_ = this->create_subscription<MsgType>(
                "/end_sub_topic", 10, std::bind(&EndNode::onPong, this, std::placeholders::_1)
            );
            noMsgs_ = 0;
        }

    private:
        void onPong(const typename MsgType::SharedPtr msg) {
            noMsgs_++;
            if (noMsgs_ > 30) {
                auto pong_received_timestamp = get_timestamp();
                uint64_t msg_delay = pong_received_timestamp - msg->info.timestamp;
                delays_in_ms_.push_back(msg_delay/1000);

                double meanSamples = mean(delays_in_ms_);
                double varianceSamples = variance(delays_in_ms_);

                stats_.push_back(std::make_tuple(meanSamples, varianceSamples));
                RCLCPP_INFO(this->get_logger(), "latency in ms: %s", 
                            std::to_string(msg_delay/1000).c_str());
                RCLCPP_INFO(this->get_logger(), "Updated mean of latency in ms: %s", 
                            std::to_string(meanSamples).c_str());
                RCLCPP_INFO(this->get_logger(), "Updated Variance of latency in ms: %s", 
                            std::to_string(varianceSamples).c_str());
                RCLCPP_INFO(this->get_logger(), "Msgs: %s", std::to_string(noMsgs_).c_str());
            }
        }
        typename rclcpp::Subscription<MsgType>::SharedPtr subscription_;

        std::vector<uint64_t> delays_in_ms_;
        std::vector<std::tuple<double, double>> stats_;
        int noMsgs_;
};

using namespace ping_pong_interfaces::msg;
template <template<class> class NodeType>
std::shared_ptr<rclcpp::Node> createNode(
    const std::string& msgType,
    const rclcpp::NodeOptions& nodeOpts = rclcpp::NodeOptions()) {
    if (msgType == "100b") 
        return std::make_shared<NodeType<Stamped100b>>(nodeOpts);
    else if (msgType == "1kb") 
        return std::make_shared<NodeType<Stamped1kb>>(nodeOpts);
    else if (msgType == "10kb")
        return std::make_shared<NodeType<Stamped10kb>>(nodeOpts);
    else if (msgType == "100kb")
        return std::make_shared<NodeType<Stamped100kb>>(nodeOpts);
    else if (msgType == "500kb")
        return std::make_shared<NodeType<Stamped500kb>>(nodeOpts);
    else 
        return std::make_shared<NodeType<Stamped100b>>(nodeOpts);
}