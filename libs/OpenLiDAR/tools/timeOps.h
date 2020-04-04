#pragma once

#include <time.h>
// #include <string>

#include "textOps.h"

static struct timespec time_start;
inline double getElapsedSeconds() {
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

inline std::string toMMSS(int _secs) {
    int hour = _secs/3600;
    _secs = _secs%3600;
    int min = _secs/60;
    _secs = _secs%60;
    int sec = _secs;
    return toString(min, 0, 2, '0') + ":" + toString(sec, 0, 2, '0');
}

inline std::string toHHMMSS(int _secs) {
    int hour = _secs/3600;
    _secs = _secs%3600;
    int min = _secs/60;
    _secs = _secs%60;
    int sec = _secs;
    return toString(hour, 0, 2, '0') + ":" + toString(min, 0, 2, '0') + ":" + toString(sec, 0, 2, '0');
}