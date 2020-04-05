
#include <string>
#include <sstream>
#include <unistd.h>

#include "drivers/mount/Celestron.h"

int main(int argc, char **argv){
    char* port = NULL;

    MountDriver* mount = new Celestron();

    if (argc > 1)
        port = argv[1];

    if (!port)
        port = mount->getPort(true);

    std::cout << "Connecting Celestron mount at: " << port << std::endl;

    if (mount->connect(port, true)) {
        std::cout << "SUCESS connecting to " << port << std::endl;
        usleep(10000);
        std::cout << "Az: " << mount->getAz() << std::endl;
        std::cout << "Alt: " << mount->getAlt() << std::endl;
        std::cout << "Offset: " << mount->getOffset().x << "," << mount->getOffset().y << "," << mount->getOffset().z << std::endl;
        mount->disconnect();
    }
    else 
        std::cout << "FAIL connecting to " << port << std::endl;

    return 0;
}
