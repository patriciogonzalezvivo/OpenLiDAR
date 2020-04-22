# TESTS

add_executable (ol_celestron ol_celestron.cpp ../libs/OpenLiDAR/drivers/mount/Celestron.cpp)
target_link_libraries (ol_celestron)

add_executable (ol_rplidar ol_rplidar.cpp ${RPLIDAR_SDK_SRC} ../libs/OpenLiDAR/drivers/lidar/RPLidar.cpp)
target_link_libraries (ol_rplidar ${CMAKE_THREAD_LIBS_INIT})

add_executable (ol_ports ol_ports.cpp ${RPLIDAR_SDK_SRC} ${OPENLIDAR_SRC})
target_link_libraries (ol_ports ${GPS_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

add_executable (ol_gpsd ol_gpsd.cpp ../libs/OpenLiDAR/drivers/gps/Gpsd.cpp)
target_link_libraries (ol_gpsd ${GPS_LIBRARY})

add_executable (ol_berryimu ol_berryimu.cpp ../libs/OpenLiDAR/drivers/imu/BerryIMU.cpp)
target_link_libraries (ol_berryimu)