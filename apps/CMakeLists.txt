
# PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (3dscan "3dscan/main.cpp" ${OPENLIDAR_SOURCE} )
target_link_libraries (3dscan ${PCL_LIBRARIES} OpenLiDAR)

add_executable (calibrate "calibrate/main.cpp" ${OPENLIDAR_SOURCE} )
target_link_libraries (calibrate OpenLiDAR)

add_executable (convert_pcl "convert_pcl/main.cpp" )
target_link_libraries (convert_pcl ${PCL_LIBRARIES} OpenLiDAR)


if (NO_X11 OR FORCE_GBM)
    add_compile_options(-std=c++14 -DGLM_FORCE_CXX14 -fpermissive -Wno-psabi)
endif()

add_executable(3dscanx "3dscanx/main.cpp" ${OPENLIDAR_SOURCE} )

set_target_properties(3dscanx PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
)
target_link_libraries (3dscanx PRIVATE
    ${PCL_LIBRARIES}
    OpenLiDAR
    ada
)

install(TARGETS 3dscan calibrate convert_pcl
        RUNTIME DESTINATION bin)