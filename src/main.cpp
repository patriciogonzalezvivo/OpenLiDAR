
#include <string>
#include <fstream>
#include <iostream>

#include "celestronprotocol.h"

// Main program
//============================================================================
int main(int argc, char **argv){
    if (ConnectTel("/dev/ttyUSB0") == -1) {
        std::cerr << "Can't find NexStar Mount in /dev/ttyUSB0" << std::endl;
        return 0;
    }

    
    

    return 0;
}