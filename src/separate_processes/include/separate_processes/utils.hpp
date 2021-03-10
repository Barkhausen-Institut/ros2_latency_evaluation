#include <time.h>
#include <vector>
#include <numeric>


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
