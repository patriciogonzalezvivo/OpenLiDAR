
#include <string>
#include <sstream>
#include <unistd.h>

#include "drivers/imu/BerryIMU.h"    

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

int main(int argc, char **argv) {

    std::cout << "Connecting..." << std::endl;
    ImuDriver* imu = new BerryIMU();
    imu->connect("/dev/i2c-%d", true);
    imu->start(true);
 
    signal(SIGINT, ctrlc);

    while (1) {
        imu->update();

        std::cout << " --- " << std::endl;
        std::cout << " Acc: " << imu->getAcc().x << " " << imu->getAcc().y << " " << imu->getAcc().z << std::endl;
        std::cout << " Gyr: " << imu->getGyr().x << " " << imu->getGyr().y << " " << imu->getGyr().z << std::endl;
        std::cout << " Heading: " << imu->getHeading() << std::endl;

        if (ctrl_c_pressed)
            break;
    }

    imu->stop(true);
    imu->disconnect();

    return 0;
}