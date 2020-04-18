
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
        std::cout << " SUCESS connecting to " << port << std::endl;
        std::cout << " Az: " << mount->getAz() << " Alt: " << mount->getAlt() << std::endl;
        std::cout << " Offset: " << mount->getOffset().x << "," << mount->getOffset().y << "," << mount->getOffset().z << std::endl;
    }
    else 
    {
        std::cout << " FAIL connecting to " << port << std::endl;
        delete mount;
        mount = NULL;
    }

    std::cout << "Connecting BerryUMI over i2c" << std::endl;
    ImuDriver* imu = new BerryIMU();
    imu->connect("/dev/i2c-%d", true);
    imu->start(true);

    bool first_line = true;
    const std::string deleteLine = "\e[2K\r\e[1A";
    std::cout << "Starting calibrating..." << std::endl;
    {
        double target = 355.0;
        double start_time = getElapsedSeconds();
        imu->start(true);
        imu->calibrate(true);
        if (mount) {
            mount->pan(target, .9, [&](double _az, double _alt) {

                // Clean prev print
                if (!first_line)
                    for (int i = 0; i < 4; i++)
                        std::cout << deleteLine;
                first_line = false;
                
                // track progress 
                int pct = (_az/target) * 100;
                float delta_time = float(getElapsedSeconds() - start_time);
                std::cout << " [ ";
                for (int i = 0; i < 50; i++) {
                    if (i < pct/2) std::cout << "#";
                    else std::cout << ".";
                }
                std::cout << " ] " << toMMSS(delta_time) << " Az: " << toString(_az,1,3,'0') << std::endl;

                // print values
                imu->update();
                std::cout << " Acc: " << imu->getAcc().x << " " << imu->getAcc().y << " " << imu->getAcc().z << std::endl;
                std::cout << " Gyr: " << imu->getGyr().x << " " << imu->getGyr().y << " " << imu->getGyr().z << std::endl;
                std::cout << " Pitch: " << imu->getPitch() << " Roll: " << imu->getRoll() << " Heading: " << imu->getHeading() << std::endl;

                return true;
            });
        }
        else {
            imu->update();
            std::cout << " Acc: " << imu->getAcc().x << " " << imu->getAcc().y << " " << imu->getAcc().z << std::endl;
            std::cout << " Gyr: " << imu->getGyr().x << " " << imu->getGyr().y << " " << imu->getGyr().z << std::endl;
            std::cout << " Pitch: " << imu->getPitch() << " Roll: " << imu->getRoll() << " Heading: " << imu->getHeading() << std::endl;
        }
        imu->calibrate(false);
    }

    {
        first_line = true;
        double start_time = getElapsedSeconds();
        double start_az = mount->getAz();
        double prev_az = 0.0;
        double prev_heading = 0.0;
        
        if (mount) {
            mount->pan(0.0, -0.9, [&](double _az, double _alt) {

                // Clean prev print
                if (!first_line)
                    for (int i = 0; i < 4; i++)
                        std::cout << deleteLine;
                first_line = false;
                
                // display progress
                int pct = (1.0-_az/start_az) * 100;
                float delta_time = float(getElapsedSeconds() - start_time);
                std::cout << " [ ";
                for (int i = 0; i < 50; i++) {
                    if (i < pct/2) std::cout << "#";
                    else std::cout << ".";
                }
                std::cout << " ] " << toMMSS(delta_time) << std::endl;

                // Compute deltas
                imu->update();
                double delta_az = prev_az - _az;
                double delta_heading = prev_heading - imu->getHeading();

                // Print results
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