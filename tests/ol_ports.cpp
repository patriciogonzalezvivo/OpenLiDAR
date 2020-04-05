
#include <string>
#include <sstream>
#include <unistd.h>

#include "OpenLiDAR.h"

int main(int argc, char **argv){
    
    OpenLiDAR scanner;
    OpenLiDARSettings settings;

    scanner.connect(settings, true);
    usleep(10000);
    scanner.disconnect();

    return 0;
}
