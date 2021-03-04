#include <time.h>

uint64_t get_timestamp() {
    long int ns;
    uint64_t all;
    time_t sec;
    struct timespec spec;

    clock_gettime(CLOCK_MONOTONIC, &spec);
    sec = spec.tv_sec;
    ns = spec.tv_nsec;

    all = (uint64_t) sec * 1000000000UL + (uint64_t) ns;
    
    return all;
}