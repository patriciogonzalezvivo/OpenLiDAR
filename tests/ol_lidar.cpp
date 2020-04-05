
#include <string>
#include <sstream>
#include <unistd.h>

#include "drivers/lidar/RPLidar.h"

int main(int argc, char **argv){
    char* port = NULL;

    LidarDriver* lidar = new RPLidar();

    if (argc > 1)
        port = argv[1];

    if (!port)
        port = lidar->getPort(true);

    std::cout << "Connecting RPLidar found at: " << port << std::endl;

    if (lidar->connect(port, true)) {
        std::cout << "SUCESSFULLY connected to " << port << std::endl;
        usleep(10000);
        lidar->disconnect();
    }

    return 0;
}
