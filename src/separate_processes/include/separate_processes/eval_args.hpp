#pragma once

#include "cxxopts/include/cxxopts.hpp"
#include <iostream>

class EvalArgs {
    public:
        EvalArgs() {
            pubFrequency = 0.;
            noNodes = 1;

            resultsDirectoryPath = "";
        }

        EvalArgs(int argc, char* argv[]) : EvalArgs() {
            parse(argc, argv);
            createResultsDirectoryPath();
        }

        void parse (int argc, char* argv[]) {
            cxxopts::Options options(argv[0], "ROS2 performance benchmark in separate processes");
            options.add_options()
                ("n,no-nodes", "Number of Nodes",
                    cxxopts::value<uint>(noNodes))
                ("f,publisher-frequency", "Publisher Frequency of start node",
                    cxxopts::value<float>(pubFrequency))
                ("h,help", "Print usage")
            ;
            auto result = options.parse(argc, argv);

            if (result.count("help")) {
                std::cout << options.help() << std::endl;
                exit(0);
            }
        }

        void print() {
            std::cout << "Arguments are set as follows:" << std::endl;
            std::cout << "Publisher frequency in Hz: " << pubFrequency << std::endl;
            std::cout << "Number of nodes between start und end node: " << noNodes << std::endl;
            std::cout << "Files will be saved to: ./" << resultsDirectoryPath << std::endl;
        }

        float pubFrequency;
        uint noNodes;
        std::string resultsDirectoryPath;

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
            strftime(timestamp, 100, "%Y-%m-%d_%H-%M", timeinfo);

            ss << timestamp << "_";
            ss << noNodes << "Nodes_" << pubFrequency << "Hz";
            resultsDirectoryPath = ss.str();
        }
};