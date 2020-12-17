
#include <string>
#include <sstream>
#include <unistd.h>

#include "mary/mount/Celestron.h"
#include "mary/imu/BerryIMU.h"
#include "mary/gps/Gpsd.h"

#include "mary/tools/timeOps.h"
#include "mary/tools/textOps.h"

int main(int argc, char **argv) {
    char* port = NULL;

    Celestron* mount = new Celestron();

    if (argc > 1)
        port = argv[1];

    if (!port)
        port = mount->getPort(true);

    std::cout << "Connecting Celestron over serial" << port << std::endl;
    if (!mount->connect(port, true))
        return -1;

    std::cout << "Connecting BerryUMI over i2c" << std::endl;
    ImuDriver* imu = new BerryIMU();
    if (!imu->connect("/dev/i2c-%d", true))
        return -1;

    std::cout << "Connecting GPS on localhost" << std::endl;
    GpsDriver* gps = new Gpsd();
    if (!gps->connect("localhost", true))
        return -1;

    bool first_line = true;
    const std::string deleteLine = "\e[2K\r\e[1A";
    std::cout << "Starting calibration" << std::endl;
    {
        double target = 355.0;
        double start_time = getElapsedSeconds();

        imu->calibrate(true);
        if (mount) {
            mount->pan(target, .9, [&](double _az, double _alt) {

                // Clean prev print
                if (!first_line)
                    for (int i = 0; i < 3; i++)
                        std::cout << deleteLine;
                first_line = false;
                
                // track progress 
                int pct = (_az/target) * 100;
                float delta_time = float(getElapsedSeconds() - start_time);
                std::cout << " Mount - [ ";
                for (int i = 0; i < 50; i++) {
                    if (i < pct/2) std::cout << "#";
                    else std::cout << ".";
                }
                std::cout << " ] " << toMMSS(delta_time) << " Az: " << toString(_az,1,3,'0') << " Alt: " << toString(_alt,1,3,'0') << std::endl;

                // print values
                imu->update();
                std::cout << " IMU   - Pitch: " << imu->getPitch() << " Roll: " << imu->getRoll() << " Heading: " << imu->getHeading() << std::endl;

                gps->update();
                std::cout << " GPS   - Lng: " << gps->getLng() << " Lat: " << gps->getLat() << " Alt: " << gps->getAlt() << std::endl;

                return true;
            });
        }
        imu->calibrate(false);
    }

    {
        first_line = true;
        double start_time = getElapsedSeconds();
        double start_az = mount->getAz();
        double prev_az = 0.0;
        double prev_alt = 0.0;
        double prev_heading = 0.0;
        double prev_offset = 0.0;
        
        if (mount) {
            mount->pan(0.0, -0.9, [&](double _az, double _alt) {

                // Clean prev print
                if (!first_line)
                    for (int i = 0; i < 6; i++)
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
                double offset = _az - imu->getHeading();

                prev_az = _az;
                prev_alt = _alt;
                prev_heading = imu->getHeading();
                prev_offset = offset;

                // Print results
                std::cout << " Mount -      Az: " << toString(_az,1,3,'0') << " delta: " << delta_az << std::endl;
                std::cout << " IMU   - Heading: " << toString(imu->getHeading(),1,3,'0') << " delta: " << delta_heading << std::endl;
                std::cout << "          Offset: " << toString(offset, 3, 3,'0') << std::endl;
                std::cout << "           Error: " << fabs(delta_az - delta_heading) << std::endl;

                gps->update();
                std::cout << " GPS -       Lng: " << toString(gps->getLng(),1,3,'0')  << " Lat: " << toString(gps->getLat(),1,3,'0') << " Alt: " << toString(gps->getAlt(),1,3,'0') << std::endl;

                return true;
            });
        }

        mount->setLocation(gps->getLng(), gps->getLat());

        double az, alt;
        mount->getAzAlt(&az, &alt);

        az += prev_offset;
        if (az < 0.0)
            az += 360.0;
        else if (az > 360.0)
            az -= 360.0;

        mount->gotoAzAlt(az, alt);
    }

    imu->disconnect();

    return 0;
}