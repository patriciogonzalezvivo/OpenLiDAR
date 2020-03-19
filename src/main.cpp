
#include <string>
#include <fstream>
#include <iostream>

#include <unistd.h>

#include "Celestron.h"


// Main program
//============================================================================
int main(int argc, char **argv){

    Celestron mount;
    if (!mount.connect("/dev/ttyUSB0")) {
        std::cerr << "Can't find NexStar Mount in /dev/ttyUSB0" << std::endl;
        return 0;
    }

    // FirmwareInfo firmware;
    // mount.getFirmware(&firmware);

    // mount.move(CELESTRON_E, SR_6);

    mount.slewAzAlt(0,0);
    double az, alt;
    while (mount.isSlewing()) {
        usleep(1000);
        mount.getAzAlt(&az, &alt);
        std::cout << "az:  " << az << " alt: " << alt << std::endl;
    }

    // mount.stop(CELESTRON_E);

    return 0;
}