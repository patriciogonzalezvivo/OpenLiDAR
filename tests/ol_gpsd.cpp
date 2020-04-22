
#include <string>
#include <sstream>
#include <unistd.h>

#include "drivers/gps/Gpsd.h"
#include "tools/timeOps.h"

int main(int argc, char **argv){
    
    std::cout << "Connecting..." << std::endl;
    GpsDriver* gps = new Gpsd();
    gps->connect("localhost", true);

    double start_time = getElapsedSeconds();
    int total = 10000;
    for (int j = 0; j < total; j++) {
        gps->update();

        const std::string deleteLine = "\e[2K\r\e[1A";
        std::cout << deleteLine;

        float time = float(getElapsedSeconds() - start_time);
        
        std::cout << " [ ";
        for (int i = 0; i < 50; i++) {
            float pct = j / float(total);
            if (i < int(pct*50)) std::cout << "#";
            else std::cout << ".";
        }
        std::cout << " ] " << toMMSS(time) << std::endl;
        usleep(1000);
    }

    std::cout << "lat: " << gps->getLat() << std::endl;
    std::cout << "lng: " << gps->getLng() << std::endl;
    std::cout << "alt: " << gps->getAlt() << "m" << std::endl;

    return 0;
}
