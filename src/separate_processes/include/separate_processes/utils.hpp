#pragma once

#include <time.h>
#include <vector>
#include <numeric>
#include "cxxopts/include/cxxopts.hpp"

double mean(std::vector<uint64_t> samples) {
    if (samples.size() == 0)
        return 0;

    double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    return sum / samples.size();
}

double variance(std::vector<uint64_t> samples) {
    if (samples.size() <= 1)
        return 0;

    double samplesMean = mean(samples);
    double sqSum = 0;
    for (int i = 0; i < samples.size(); i++) {
        sqSum += (samples.at(i) * samples.at(i));
    }
    return (sqSum/(samples.size()-1) - samplesMean * samplesMean);
}

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


/*static uint64_t get_timestamp() {
  long int ns;
  uint64_t all;
  time_t sec;
  struct timespec spec;

  clock_gettime(CLOCK_MONOTONIC_RAW, &spec);
  sec = spec.tv_sec;
  ns = spec.tv_nsec;

  all = (uint64_t) sec * 1000000000UL + (uint64_t) ns;

  return all;
}*/
