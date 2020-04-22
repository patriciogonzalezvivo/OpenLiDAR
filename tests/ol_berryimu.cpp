
#include <string>
#include <sstream>
#include <unistd.h>

#include "drivers/imu/BerryIMU.h"

#include "tools/timeOps.h"
#include "tools/textOps.h"

#include <signal.h>
bool bRun = true;
void CtrlC(int) {
    bRun = false;
}

int main(int argc, char **argv) {
    std::cout << "Connecting BerryUMI over i2c" << std::endl;
    ImuDriver* imu = new BerryIMU();
    imu->connect("/dev/i2c-%d", true);

    bRun = true;
    signal(SIGINT, CtrlC );

    bool first_line = true;
    const std::string deleteLine = "\e[2K\r\e[1A";
    while (bRun) {
        if (!first_line)
            for (int i = 0; i < 3; i++)
                std::cout << deleteLine;
        first_line = false;

        imu->update();
        std::cout << " Acc: " << imu->getAcc().x << " " << imu->getAcc().y << " " << imu->getAcc().z << std::endl;
        std::cout << " Gyr: " << imu->getGyr().x << " " << imu->getGyr().y << " " << imu->getGyr().z << std::endl;
        std::cout << " Pitch: " << imu->getPitch() << " Roll: " << imu->getRoll() << " Heading: " << imu->getHeading() << std::endl;
        usleep(1000);
    }
    imu->calibrate(false);
    
    imu->disconnect();

    return 0;
}