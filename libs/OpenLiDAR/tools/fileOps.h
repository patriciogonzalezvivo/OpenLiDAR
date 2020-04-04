#pragma once

#include <string>
#include <fstream>

bool doFileExist(const char *_fileName) {
    std::ifstream infile(_fileName);
    return infile.good();
}

std::string getUniqueFileName( const std::string& _originalName, const std::string& _extension) {
    std::string filename = _originalName + "." + _extension;
    int index = 0;
    while ( doFileExist( filename.c_str() ) ) {
        filename = _originalName + "_" + toString(index, 3, '0') + "." + _extension;
        index++;
    }
    return filename;
}
