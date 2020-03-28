
#pragma once

#include <string>
#include <iomanip>
#include <sstream>

template <class T>
inline std::string toString(const T& _value, int _precision, int _width, char _fill) {
    std::ostringstream out;
    out << std::fixed << std::setfill(_fill) << std::setw(_width) << std::setprecision(_precision) << _value;
    return out.str();
}

double getElapsedSeconds();
std::string toMMSS(int _secs);
std::string toHHMMSS(int _secs);
