#pragma once

#include "cxxopts/include/cxxopts.hpp"
#include <iostream>
#include <cstdlib>

#include "json.hpp"

class EvalArgs {
public:
    EvalArgs() {}

    EvalArgs(int argc, char* argv[]) {
        parse(argc, argv);
        createResultsDirectoryPath();
        createResultFilename();
    }

    void parse (int argc, char* argv[]) {
        cxxopts::Options options(argv[0], "ROS2 performance benchmark in separate processes");
        options.allow_unrecognised_options();
        options.add_options()
        ("prefix", "Prefix for the result directory name",
            cxxopts::value<std::string>(dirPrefix))
        ("N,no-nodes", "Number of Nodes",
            cxxopts::value<uint>(noNodes))
        ("f,publisher-frequency", "Publisher Frequency of start node",
            cxxopts::value<float>(pubFrequency))
        ("m,msg-size", "Size of msg, Supported: 128b, 1kb, 10kb, 100kb, 500kb",
            cxxopts::value<std::string>(msgSize))
        ("n,node-index", "Index of the node in the chain. 0 = start node, N-1 = end node",
            cxxopts::value<uint>(nodeIndex))
        ("d,duration", "Duration (in seconds) of the measurement",
            cxxopts::value<uint>(duration))
        ("q,qos", "QOS profile (best-effort or reliable)",
            cxxopts::value<std::string>(qos))
        ("h,help", "Print usage")
        ;

        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            std::cout << options.help() << std::endl;
            exit(0);
        }

        verifyArgs();
    }

    void print() {
        std::cout << "Arguments are set as follows:" << std::endl;
        std::cout << "Publisher frequency in Hz: " << pubFrequency << std::endl;
        std::cout << "Number of nodes including start and end node: " << noNodes << std::endl;
        std::cout << "Msg size: " << msgSize << std::endl;
        std::cout << "Measurement duration: " << duration << std::endl;
        std::cout << "QOS profile: " << qos << std::endl;
        std::cout << "Files will be saved to: ./" << resultsDirectoryPath << std::endl;
        std::cout << "Into file " << resultsFilename << std::endl;
	std::cout << "RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE:" << std::getenv("
RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE") << std::endl;
    }

    void dumpJsonIntoResultDirectory() {
        using json = nlohmann::json;
        json j;
        j["timestamp"] = getTimestamp();
        j["dirPrefix"] = dirPrefix;
        j["pub_frequency"] = pubFrequency;
        j["msg_size"] = msgSize;
        j["duration"] = duration;
        j["qos"] = qos;
        j["middleware"] = getMiddleware();
        j["node_index"] = nodeIndex;
        j["no_nodes"] = noNodes;
        std::cout << j.dump(4) << std::endl;

        std::ofstream of(resultsDirectoryPath + "/config.json");
        of << j.dump(4) << std::endl;

    }

    const uint NODE_NOT_SET = static_cast<uint>(-1);

    float pubFrequency = 1;
    uint noNodes = NODE_NOT_SET;
    uint nodeIndex = NODE_NOT_SET;
    uint duration = 10;
    std::string dirPrefix = "";
    std::string msgSize = "128b";
    std::string qos = "best-effort";

    std::string resultsDirectoryPath = "";
    std::string resultsFilename = "";

private:
    void createResultsDirectoryPath() {
        std::ostringstream ss;
        ss.precision(2);

        ss << "results/";
        ss << dirPrefix << "_";
        ss << noNodes << "Nodes_" << pubFrequency << "Hz_";
        ss << msgSize << "_" << getMiddleware() << "_";
        ss << qos << "QOS_";
        ss << duration << "s";
        resultsDirectoryPath = ss.str();
    }

    std::string getTimestamp() const {
        // get current time stamp
        time_t now;
        struct tm* timeinfo;
        char timestamp[100];

        time(&now);
        timeinfo = localtime(&now);
        strftime(timestamp, 100, "%Y-%m-%d_%H-%M-%S", timeinfo);

        return timestamp;
    }

    std::string getMiddleware() const {
        const char* rmw = std::getenv("RMW_IMPLEMENTATION");
        if (rmw)
            return rmw;
        else
            return "unknown";
    }

    void createResultFilename() {
        createResultsDirectoryPath();
        std::ostringstream ss;
        ss << resultsDirectoryPath << "/";
        // Use + 1 for the index to make it 1-based
        ss << nodeIndex + 1 << "-" << noNodes << ".csv";
        resultsFilename = ss.str();
    }

    void verifyArgs() {
        if (std::find(
                SUPPORTED_MSG_SIZES_.begin(),
                SUPPORTED_MSG_SIZES_.end(),
                msgSize) == SUPPORTED_MSG_SIZES_.end())
        {
            std::cout << "Message size not supported" << std::endl;
            exit(1);
        }
        if (noNodes == NODE_NOT_SET) {
            std::cerr << "Number of nodes must be given" << std::endl;
            exit(1);
        }
        if (nodeIndex == NODE_NOT_SET) {
            std::cerr << "Node index must be given" << std::endl;
            exit(1);
        }
        if (qos != "reliable" && qos != "best-effort") {
            std::cerr << "QoS Setting invalid" << std::endl;
            exit(1);
        }
        if (dirPrefix == "") {
            std::cerr << "dirPrefix not set!" << std::endl;
            exit(1);
        }
    }

    const std::array<std::string, 5> SUPPORTED_MSG_SIZES_ = {"128b", "1kb", "10kb", "100kb", "500kb"};

};
