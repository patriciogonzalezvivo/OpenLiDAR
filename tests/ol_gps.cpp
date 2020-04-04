
#include <string>
#include <sstream>
#include <unistd.h>

#include "drivers/gps/Gpsd.h"
#include "tools/timeOps.h"

int main(int argc, char **argv){
    
    std::cout << "Connecting..." << std::endl;
    GpsDriver* gps = new Gpsd();
    gps->connect("localhost", true);
    gps->start();

    double start_time = getElapsedSeconds();
    for (int pct = 0; pct < 101; pct++) {
        const std::string deleteLine = "\e[2K\r\e[1A";
        std::cout << deleteLine;

        float time = float(getElapsedSeconds() - start_time);
        
        std::cout << " [ ";
        for (int i = 0; i < 50; i++) {
            if (i < pct/2) std::cout << "#";
            else std::cout << ".";
        }
        std::cout << " ] " << toMMSS(time) << std::endl;
        usleep(500000);
    }

    gps->stop();

    std::cout << "lat: " << gps->getLat() << std::endl;
    std::cout << "lng: " << gps->getLng() << std::endl;
    std::cout << "alt: " << gps->getAlt() << "m" << std::endl;

    return 0;
}
