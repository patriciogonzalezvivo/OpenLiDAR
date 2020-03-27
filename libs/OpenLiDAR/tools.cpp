#include "tools.h"

#include <time.h>

struct timespec time_start;
double getElapsedSeconds() {
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    timespec temp;
    if ((now.tv_nsec-time_start.tv_nsec)<0) {
        temp.tv_sec = now.tv_sec-time_start.tv_sec-1;
        temp.tv_nsec = 1000000000+now.tv_nsec-time_start.tv_nsec;
    } else {
        temp.tv_sec = now.tv_sec-time_start.tv_sec;
        temp.tv_nsec = now.tv_nsec-time_start.tv_nsec;
    }
    return double(temp.tv_sec) + double(temp.tv_nsec/1000000000.);
}
