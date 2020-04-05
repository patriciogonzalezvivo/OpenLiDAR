
#include <string>
#include <sstream>
#include <unistd.h>

#include "OpenLiDAR.h"

int main(int argc, char **argv){
    OpenLiDAR scanner;
    OpenLiDARSettings settings;

    if (scanner.connect(settings, true)) {
        std::cout << "SUCESS connecting scanner " << std::endl;
        usleep(10000);
        scanner.disconnect();
    }
    else 
        std::cout << "FAIL connecting scanner " << std::endl;

    return 0;
}
