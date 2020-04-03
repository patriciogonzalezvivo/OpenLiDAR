
#include <string>
#include <sstream>
#include <unistd.h>

#include "mount/Celestron.h"

int main(int argc, char **argv){
    
    std::cout << "Connecting..." << std::endl;

    MountDriver* mount = new Celestron();

    char* port = mount->getPort();
    std::cout << "Celestron mount found at: " << port << std::endl;

    mount->connect(port, true);
    mount->printFirmware();
    usleep(500000);
    mount->disconnect();

    return 0;
}
