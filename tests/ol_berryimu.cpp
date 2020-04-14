
#include <string>
#include <sstream>
#include <unistd.h>

#include "drivers/mount/Celestron.h"
#include "drivers/imu/BerryIMU.h"

#include "tools/timeOps.h"
#include "tools/textOps.h"

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

int main(int argc, char **argv) {
    char* port = NULL;

    MountDriver* mount = new Celestron();

    if (argc > 1)
        port = argv[1];

    if (!port)
        port = mount->getPort(true);

    std::cout << "Connecting Celestron mount at: " << port << std::endl;

    if (mount->connect(port, true)) {
        std::cout << "SUCESS connecting to " << port << std::endl;
        std::cout << "Az: " << mount->getAz() << std::endl;
        std::cout << "Alt: " << mount->getAlt() << std::endl;
        std::cout << "Offset: " << mount->getOffset().x << "," << mount->getOffset().y << "," << mount->getOffset().z << std::endl;
        mount->disconnect();
    }
    else 
        std::cout << "FAIL connecting to " << port << std::endl;

    std::cout << "Connecting BerryUMI at i2c port";
    ImuDriver* imu = new BerryIMU();
    imu->connect("/dev/i2c-%d", true);
    imu->start(true);

    std::cout << "Starting calibrating..." << std::endl;

    
    double az = 0.0;
    double target = 355.0;
    double start_time = getElapsedSeconds();

    imu->calibrate(true);
    if (mount) {
        mount->start(1.0, true);
        az = mount->getAz();
    }

    signal(SIGINT, ctrlc);

    const std::string deleteLine = "\e[2K\r\e[1A";
    bool first_line = true;
    while (az < target) {

        // Update Data
        imu->update();
        if (mount) 
            az = mount->getAz();
        else 
            az += 0.5;

        // Clean prev print
        if (!first_line)
            for (int i = 0; i < 6; i++)
                std::cout << deleteLine;
        first_line = false;
        
        // Compute 
        int pct = (az/target) * 100;
        float delta_time = float(getElapsedSeconds() - start_time);
        std::cout << " [ ";
        for (int i = 0; i < 50; i++) {
            if (i < pct/2) std::cout << "#";
            else std::cout << ".";
        }
        std::cout << " ] " << toMMSS(delta_time) << std::endl;

        std::cout << " Az: " <<  toString(az,1,3,'0') << std::endl;
        std::cout << " Acc: " << imu->getAcc().x << " " << imu->getAcc().y << " " << imu->getAcc().z << std::endl;
        std::cout << " Gyr: " << imu->getGyr().x << " " << imu->getGyr().y << " " << imu->getGyr().z << std::endl;
        std::cout << " Pitch: " << imu->getPitch() << " Roll: " << imu->getRoll() << " Heading: " << imu->getHeading() << std::endl;
        std::cout << " Tmp: " << imu->getTmp().x << " " << imu->getTmp().y << std::endl;

        if (ctrl_c_pressed)
            break;

        usleep(25000);
    }

    if (mount) {
        mount->stop(true);
        mount->reset(true);
        az = mount->getAz();
    }

    imu->printFirmware();

    imu->update();
    std::cout << " Az: " <<  toString(az,1,3,'0') << std::endl;
    std::cout << " Acc: " << imu->getAcc().x << " " << imu->getAcc().y << " " << imu->getAcc().z << std::endl;
    std::cout << " Gyr: " << imu->getGyr().x << " " << imu->getGyr().y << " " << imu->getGyr().z << std::endl;
    std::cout << " Pitch: " << imu->getPitch() << " Roll: " << imu->getRoll() << " Heading: " << imu->getHeading() << std::endl;
    std::cout << " Tmp: " << imu->getTmp().x << " " << imu->getTmp().y << std::endl;
    
    imu->stop(true);
    imu->disconnect();

    return 0;
}