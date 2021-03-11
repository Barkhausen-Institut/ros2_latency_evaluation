#pragma once

#include "cxxopts/include/cxxopts.hpp"
#include <iostream>

class EvalArgs {
    public:
        EvalArgs() {
            pubFrequency = 0.;
            noNodes = 1;
        }

        EvalArgs(int argc, char* argv[]) : EvalArgs() {
            parse(argc, argv);
        }

        void parse (int argc, char* argv[]) {
            cxxopts::Options options(argv[0], "ROS2 performance benchmark in separate processes");
            options.add_options()
                ("n,no-nodes", "Number of Nodes",
                    cxxopts::value<uint>(noNodes))
                ("f,publisher-frequency", "Publisher Frequency of start node",
                    cxxopts::value<float>(pubFrequency))
            ;
        }

        void printOptions() {
            std::cout << "Arguments are set as follows:" << std::endl;
            std::cout << "Publisher frequency in Hz: " << pubFrequency << std::endl;
            std::cout << "Number of nodes between start und end node: " << noNodes << std::endl;
        }

        float pubFrequency;
        uint noNodes;
};