
# PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (3dscan "3dscan/main.cpp" ${RPLIDAR_SDK_SRC} ${OPENLIDAR_SRC} )
target_link_libraries (3dscan ${PCL_LIBRARIES} ${GPS_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} )

add_executable (calibrate "calibrate/main.cpp" ${RPLIDAR_SDK_SRC} ${OPENLIDAR_SRC} )
target_link_libraries (calibrate ${GPS_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} )

add_executable (convert_pcl "convert_pcl/main.cpp" )
target_link_libraries (convert_pcl ${PCL_LIBRARIES} )

add_executable (tile_pcl "tile_pcl/main.cpp" )
target_link_libraries (tile_pcl ${PCL_LIBRARIES} )

install(TARGETS 3dscan calibrate convert_pcl tile_pcl
        RUNTIME DESTINATION bin)