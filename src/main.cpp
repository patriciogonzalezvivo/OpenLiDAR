
#include <string>
#include <fstream>
#include <iostream>

#include "Celestron.h"


// Main program
//============================================================================
int main(int argc, char **argv){

    Celestron mount;
    if (!mount.connect("/dev/ttyUSB0")) {
        std::cerr << "Can't find NexStar Mount in /dev/ttyUSB0" << std::endl;
        return 0;
    }

    FirmwareInfo firmware;
    mount.getFirmware(&firmware);

    return 0;
}