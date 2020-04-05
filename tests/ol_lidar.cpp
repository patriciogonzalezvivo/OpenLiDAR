
#include <string>
#include <sstream>
#include <unistd.h>

#include "drivers/lidar/RPLidar.h"

int main(int argc, char **argv){
    
    std::cout << "Connecting..." << std::endl;

    LidarDriver* lidar = new RPLidar();

    char* port = lidar->getPort(true);
    std::cout << "RPLidar found at: " << port << std::endl;

    lidar->connect(port, true);
    usleep(10000);

    lidar->disconnect();

    return 0;
}
