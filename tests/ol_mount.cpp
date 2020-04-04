
#include <string>
#include <sstream>
#include <unistd.h>

#include "mount/Celestron.h"

int main(int argc, char **argv){
    
    std::cout << "Connecting..." << std::endl;

    MountDriver* mount = new Celestron();

    char* port = mount->getPort(true);
    std::cout << "Celestron mount found at: " << port << std::endl;

    mount->connect(port, true);
    mount->printFirmware();
    usleep(500000);

    std::cout << "Az: " << mount->getAz() << std::endl;
    std::cout << "Alt: " << mount->getAlt() << std::endl;
    std::cout << "Offset: " << mount->getOffset().x << "," << mount->getOffset().y << "," << mount->getOffset().z << std::endl;

    mount->disconnect();

    return 0;
}
