#pragma once

#include "cxxopts/include/cxxopts.hpp"
#include <iostream>

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
	    ("N,no-nodes", "Number of Nodes",
	     cxxopts::value<uint>(noNodes))
	    ("f,publisher-frequency", "Publisher Frequency of start node",
	     cxxopts::value<float>(pubFrequency))
	    ("m,msg-size", "Size of msg, Supported: 100b, 1kb, 10kb, 100kb, 500kb",
	     cxxopts::value<std::string>(msgSize))
	    ("n,node-index", "Index of the node in the chain. 0 = start node, N-1 = end node",
	     cxxopts::value<int>(nodeIndex))
	    ("d,duration", "Duration (in seconds) of the measurement",
	     cxxopts::value<uint>(duration))
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
	std::cout << "Files will be saved to: ./" << resultsDirectoryPath << std::endl;
	std::cout << "Into file " << resultsFilename << std::endl;
    }

    float pubFrequency = 1;
    uint noNodes = -1;
    int nodeIndex = -1;
    uint duration = 10;
    std::string resultsDirectoryPath = "";
    std::string resultsFilename = "";
    std::string msgSize = "100b";

private:
    void createResultsDirectoryPath() {
	std::ostringstream ss;
	ss.precision(2);

	// get current time stamp
	time_t now;
	struct tm* timeinfo;
	char timestamp[100];

	time(&now);
	timeinfo = localtime(&now);
	strftime(timestamp, 100, "%Y-%m-%d_%H-%M-%S", timeinfo);

	ss << "results/";
	ss << timestamp << "_";
	ss << noNodes << "Nodes_" << pubFrequency << "Hz_";
	ss << msgSize << "_" << getMiddleware() << "_";
	ss << duration << "s";
	resultsDirectoryPath = ss.str();
    }

    std::string getMiddleware() {
	// TODO: Implement MW detection
	return "unknown";
    }

    void createResultFilename() {
	createResultsDirectoryPath();
	std::ostringstream ss;
	ss << resultsDirectoryPath << "/";
	ss << nodeIndex << "-" << noNodes << ".csv";
	resultsFilename = ss.str();
    }

    void verifyArgs() {
	if (std::find(
		SUPPORTED_MSG_SIZES_.begin(),
		SUPPORTED_MSG_SIZES_.end(),
		msgSize) == SUPPORTED_MSG_SIZES_.end()) {
	    std::cout << "Message size not supported" << std::endl;
	    exit(0);
	}
	if (noNodes < 0) {
	    std::cerr << "Number of nodes must be given" << std::endl;
	    exit(1);
	}
	if (nodeIndex < 0) {
	    std::cerr << "Node index must be given" << std::endl;
	    exit(1);
	}
    }

    std::array<std::string, 5> SUPPORTED_MSG_SIZES_ = {"100b", "1kb", "10kb", "100kb", "500kb"};
};
