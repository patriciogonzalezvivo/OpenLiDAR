#pragma once

#include <string>
#include <fstream>
#include "textOps.h"

inline bool doFileExist(const char *_fileName) {
    std::ifstream infile(_fileName);
    return infile.good();
}

inline std::string getUniqueFileName( const std::string& _originalName, const std::string& _extension) {
    std::string filename = _originalName + "." + _extension;
    int index = 0;
    while ( doFileExist( filename.c_str() ) ) {
        filename = _originalName + "_" + toString(index, 3, '0') + "." + _extension;
        index++;
    }
    return filename;
}

inline std::string getExt(const std::string& _filename) {
    if (_filename.find_last_of(".") != std::string::npos)
        return _filename.substr(_filename.find_last_of(".") + 1);
    return "";
}