
#include <string>
#include <sstream>
#include <unistd.h>

#include "drivers/mount/Celestron.h"
#include "drivers/imu/BerryIMU.h"

#include "tools/timeOps.h"
#include "tools/textOps.h"

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
    {
        bool first_line = true;
        double target = 355.0;
        double start_time = getElapsedSeconds();
        const std::string deleteLine = "\e[2K\r\e[1A";
        imu->calibrate(true);
        if (mount) {
            mount->pan(target, 1.0, [&](double _az, double _alt) {

                // Clean prev print
                if (!first_line)
                    for (int i = 0; i < 5; i++)
                        std::cout << deleteLine;
                first_line = false;
                
                // Compute 
                int pct = (_az/target) * 100;
                float delta_time = float(getElapsedSeconds() - start_time);
                std::cout << " [ ";
                for (int i = 0; i < 50; i++) {
                    if (i < pct/2) std::cout << "#";
                    else std::cout << ".";
                }
                std::cout << " ] " << toMMSS(delta_time) << " Az: " << toString(_az,1,3,'0') << std::endl;

                imu->update();
                std::cout << " Acc: " << imu->getAcc().x << " " << imu->getAcc().y << " " << imu->getAcc().z << std::endl;
                std::cout << " Gyr: " << imu->getGyr().x << " " << imu->getGyr().y << " " << imu->getGyr().z << std::endl;
                std::cout << " Pitch: " << imu->getPitch() << " Roll: " << imu->getRoll() << " Heading: " << imu->getHeading() << std::endl;
                std::cout << " Tmp: " << imu->getTmp().x << " " << imu->getTmp().y << std::endl;

                return true;
            });
        }
        imu->calibrate(false);
    }

    imu->printFirmware();

    std::cout << "Return to 0 and check error" << std::endl;
    {
        bool first_line = true;
        double target = 0.0;
        double start_time = getElapsedSeconds();
        const std::string deleteLine = "\e[2K\r\e[1A";
        double prev_az = 0.0;
        double prev_heading = 0.0;
        
        if (mount) {
            mount->pan(target, -1.0, [&](double _az, double _alt) {

                // Clean prev print
                if (!first_line)
                    for (int i = 0; i < 4; i++)
                        std::cout << deleteLine;
                first_line = false;
                
                // Compute 
                int pct = (_az/target) * 100;
                float delta_time = float(getElapsedSeconds() - start_time);
                std::cout << " [ ";
                for (int i = 0; i < 50; i++) {
                    if (i < pct/2) std::cout << "#";
                    else std::cout << ".";
                }
                std::cout << " ] " << toMMSS(delta_time) << std::endl;

                double delta_az = prev_az - _az;
                double delta_heading = prev_heading - imu->getHeading();

                imu->update();
                std::cout << " Az: " << toString(_az,1,3,'0') << " delta: " << delta_az << std::endl;
                std::cout << " Heading: " << toString(imu->getHeading(),1,3,'0') << " delta: " << delta_heading << std::endl;
                std::cout << " Error: " << abs(delta_az - delta_heading) << std::endl;
                
                return true;
            });
        }
    }

    imu->stop(true);
    imu->disconnect();

    return 0;
}