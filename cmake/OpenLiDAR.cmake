
# GLM
find_package(glm REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})
link_directories(${GLM_LIBRARY_DIRS})
add_definitions(${GLM_DEFINITIONS})

# GPS
find_path(GPS_INCLUDE_DIR NAMES libgpsmm.h)
find_library(GPS_LIBRARY NAMES gps)
if(NOT GPS_INCLUDE_DIR OR NOT GPS_LIBRARY)
    MESSAGE(FATAL_ERROR "Could not find GPS library")
endif(NOT GPS_INCLUDE_DIR OR NOT GPS_LIBRARY)
message(STATUS "Found GPS: ${GPS_LIBRARY}")

# Threads
find_package (Threads)

# OPENLIDAR
set(OPENLIDAR_PATH "libs/OpenLiDAR")
FILE(GLOB OPENLIDAR_SRC
    "${OPENLIDAR_PATH}/*.cpp"
    "${OPENLIDAR_PATH}/*/*.cpp"
    "${OPENLIDAR_PATH}/*/*/*.cpp"
)
include_directories(
    ${OPENLIDAR_PATH}
)

# RPLIDAR DRIVER
set(RPLIDAR_SDK_PATH "${OPENLIDAR_PATH}/drivers/lidar/rplidar_sdk")
FILE(GLOB RPLIDAR_SDK_SRC 
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
)
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
)
