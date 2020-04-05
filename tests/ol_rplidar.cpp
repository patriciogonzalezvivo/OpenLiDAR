
#include <string>
#include <sstream>
#include <unistd.h>

#include "drivers/lidar/RPLidar.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

void plot_histogram(LidarSample* nodes, size_t count) {
    const int BARCOUNT =  75;
    const int MAXBARHEIGHT = 20;
    const float ANGLESCALE = 360.0f/BARCOUNT;

    float histogram[BARCOUNT];
    for (int pos = 0; pos < _countof(histogram); ++pos)
        histogram[pos] = 0.0f;

    float max_val = 0;
    for (int pos =0 ; pos < (int)count; ++pos) {
        int int_deg = (int)(nodes[pos].theta);
        if (int_deg >= BARCOUNT) int_deg = 0;
        float cachedd = histogram[int_deg];
        if (cachedd == 0.0f )
            cachedd = nodes[pos].distance;
        else
            cachedd = (nodes[pos].distance + cachedd)*0.5f;

        if (cachedd > max_val) max_val = cachedd;
        histogram[int_deg] = cachedd;
    }

    for (int height = 0; height < MAXBARHEIGHT; ++height) {
        float threshold_h = (MAXBARHEIGHT - height - 1) * (max_val/MAXBARHEIGHT);
        for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
            if (histogram[xpos] >= threshold_h) {
                putc('*', stdout);
            }else {
                putc(' ', stdout);
            }
        }
        printf("\n");
    }
    for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
        putc('-', stdout);
    }
    printf("\n");
}

int main(int argc, char **argv){
    char* port = NULL;

    LidarDriver* lidar = new RPLidar();

    if (argc > 1)
        port = argv[1];

    if (!port)
        port = lidar->getPort(true);

    std::cout << "Connecting RPLidar found at: " << port << std::endl;

    if (lidar->connect(port, true)) {
        std::cout << "SUCESS connecting to " << port << std::endl;
        lidar->start(true);

        size_t count = 0;
        LidarSample samples[RPLIDAR_MAXSAMPLES];
        lidar->getSamples(samples, count);
        plot_histogram(samples, count);

        usleep(10000);
        lidar->stop(true);
        lidar->disconnect();
    }
    else 
        std::cout << "FAIL connecting to " << port << std::endl;

    return 0;
}
