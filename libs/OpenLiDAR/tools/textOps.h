
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

/// like sprintf "% 4d" or "% 4f" format, in this example width=4, fill=' '
template <class T>
inline std::string toString(const T& _value, int _width, char _fill) {
    std::ostringstream out;
    out << std::fixed << std::setfill(_fill) << std::setw(_width) << _value;
    return out.str();
}

inline float toFloat(const std::string& _string) {
    float x = 0;
    std::istringstream cur(_string);
    cur >> x;
    return x;
}