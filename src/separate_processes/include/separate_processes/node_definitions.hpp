#pragma once

#include <chrono>
#include <string>
#include <vector>
#include <tuple>
#include <ctime>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "ros2profiling/profiling.h"
#include "ping_pong_interfaces/msg/stamped128b.hpp"
#include "ping_pong_interfaces/msg/stamped1kb.hpp"
#include "ping_pong_interfaces/msg/stamped10kb.hpp"
#include "ping_pong_interfaces/msg/stamped100kb.hpp"
#include "ping_pong_interfaces/msg/stamped500kb.hpp"

#include "utils.hpp"
#include "eval_args.hpp"

using namespace std::chrono_literals;

const uint NUM_PROFILING_STEPS = 14;

class BenchmarkNode : public rclcpp::Node {
public:
    BenchmarkNode(const std::string& nodeName,
                    const EvalArgs& args,
                    const rclcpp::NodeOptions& opt) 
    : Node(nodeName, "", opt), args_(args) {
        startTime = std::chrono::system_clock::now();
        shutdownTimer_ = create_wall_timer(std::chrono::seconds(1),
                                            [this]() { onShutdownTimer(); });

        createResultFile();
   }

protected:
   void onShutdownTimer() {
        auto now = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = (now - startTime);
        int offset = 0;
        if (args_.nodeIndex > 0)
            offset = 2; // kill the following nodes a bit later than the start node.
        if (diff.count()  > args_.duration + offset) {
            std::cout << "Shutting down" << std::endl;
            rclcpp::shutdown();
        }
   }

   void createResultFile() {
        resultDump_.open(args_.resultsFilename);
        resultDump_ << "tracking_number,";
        resultDump_ << "header_timestamp,";
        for (int i = 0; i < NUM_PROFILING_STEPS; i++) {
            resultDump_ << "prof_" + std::to_string(i) << ",";
        }
        resultDump_ << "callback_timestamp";
        resultDump_ << std::endl;
   }

    template<class Msg>
    void dumpTimestamps(const typename Msg::SharedPtr msg, uint64_t callbackTimestamp) {
        const void* rawMsg = msg.get();
        resultDump_ << msg->info.tracking_number << ",";
        resultDump_ << msg->info.timestamp << ",";
        for (int i = 0; i < NUM_PROFILING_STEPS; i++) {
            resultDump_ << get_profile(rawMsg, i) << ",";
        }
        resultDump_ << callbackTimestamp;
        resultDump_ << std::endl;
    }

    std::ofstream resultDump_;

    EvalArgs args_;
    std::chrono::time_point<std::chrono::system_clock> startTime;
    rclcpp::TimerBase::SharedPtr shutdownTimer_;
};

template <class MsgType>
class StartNode : public BenchmarkNode {
    public:
        StartNode(
            const EvalArgs& args,
            const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
        : BenchmarkNode("start_node", args, opt)
        {
            uint32_t pubPeriodMs = static_cast<uint32_t>(1/args_.pubFrequency * 1000);
            RCLCPP_INFO(this->get_logger(), "Publishing every %d ms", pubPeriodMs);
            publisher_ = this->create_publisher<MsgType>("/start_pub_topic", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(pubPeriodMs),
                std::bind(&StartNode::timer_callback, this));
        }

    private:
        void timer_callback() {
            auto msg = MsgType();
            auto now = get_timestamp();
            msg.info.timestamp = now;
            msg.info.tracking_number = trackingNumber_++;
            publisher_->publish(msg);

            if (trackingNumber_ % 100 == 0)
                RCLCPP_INFO(get_logger(), "Published %d messages", trackingNumber_);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
        uint trackingNumber_ = 0;
};

template <class MsgType>
class IntermediateNode : public BenchmarkNode {
    public:
        IntermediateNode(
            const EvalArgs& args,
            const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
       : BenchmarkNode("intermediate_node", args, opt)
        {
            publisher_ = this->create_publisher<MsgType>("/end_sub_topic", 10);
            subscription_ = this->create_subscription<MsgType>(
                "/start_pub_topic", 10, std::bind(&IntermediateNode::onPing, this, std::placeholders::_1)
            );
        }

    private:
        void onPing(const typename MsgType::SharedPtr msg) {
            dumpTimestamps<MsgType>(msg, get_timestamp());
            publisher_->publish(*msg);
        }
        typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
        typename rclcpp::Subscription<MsgType>::SharedPtr subscription_;
};

template <class MsgType>
class EndNode : public BenchmarkNode {
    public:
        EndNode(
            const EvalArgs& args,
            const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
        : BenchmarkNode("end_node", args, opt)
        {
            subscription_ = this->create_subscription<MsgType>(
                "/end_sub_topic", 10, std::bind(&EndNode::onPong, this, std::placeholders::_1)
            );
            noMsgs_ = 0;
        }

    private:
        void onPong(const typename MsgType::SharedPtr msg) {
            uint64_t callbackTimestamp = get_timestamp();
            dumpTimestamps<MsgType>(msg, callbackTimestamp);
            noMsgs_++;
            if (noMsgs_ % 100 == 0) {
                uint64_t latency = callbackTimestamp - msg->info.timestamp;
                RCLCPP_INFO(get_logger(), "Received %d messages. Last latency: %llu",
                            noMsgs_, latency);
            }
        }
        typename rclcpp::Subscription<MsgType>::SharedPtr subscription_;

        int noMsgs_;
};

using namespace ping_pong_interfaces::msg;
template <template<class> class NodeType>
std::shared_ptr<rclcpp::Node> createNode(
    const EvalArgs& args,
    const rclcpp::NodeOptions& nodeOpts = rclcpp::NodeOptions()) {
    if (args.msgSize == "128b")
        return std::make_shared<NodeType<Stamped128b>>(args, nodeOpts);
    else if (args.msgSize == "1kb")
        return std::make_shared<NodeType<Stamped1kb>>(args, nodeOpts);
    else if (args.msgSize == "10kb")
        return std::make_shared<NodeType<Stamped10kb>>(args, nodeOpts);
    else if (args.msgSize == "100kb")
        return std::make_shared<NodeType<Stamped100kb>>(args, nodeOpts);
    else if (args.msgSize == "500kb")
        return std::make_shared<NodeType<Stamped500kb>>(args, nodeOpts);
    else
        throw std::invalid_argument("Msg size not supported");
}
