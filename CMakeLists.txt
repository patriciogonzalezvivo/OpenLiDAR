cmake_minimum_required(VERSION 3.1)

if(${CMAKE_VERSION} VERSION_LESS 3.15)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
else()
    cmake_policy(VERSION 3.15)
endif()

project(OpenLiDAR VERSION 0.1
                  DESCRIPTION "Open 3D LiDAR Scanner "
                  LANGUAGES CXX)

add_definitions(-DDEBUG_USING_SIMULATE_DATA)

# PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# GLM
find_package(glm REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})
link_directories(${GLM_LIBRARY_DIRS})
add_definitions(${GLM_DEFINITIONS})

# RPLIDAR DRIVER
set(RPLIDAR_SDK_PATH "libs/rplidar_sdk")
FILE(GLOB RPLIDAR_SDK_SRC 
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
)
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
)

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

# APPS
add_executable (3dscan "apps/3dscan/main.cpp" ${RPLIDAR_SDK_SRC} ${OPENLIDAR_SRC} )
target_link_libraries (3dscan ${PCL_LIBRARIES} ${GPS_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

# TESTS
add_executable (ol_gps "tests/ol_gps.cpp" ${RPLIDAR_SDK_SRC} ${OPENLIDAR_SRC} )
target_link_libraries (ol_gps ${GPS_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

add_executable (ol_mount "tests/ol_mount.cpp" ${RPLIDAR_SDK_SRC} ${OPENLIDAR_SRC})
target_link_libraries (ol_mount ${GPS_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

add_executable (ol_lidar "tests/ol_lidar.cpp" ${RPLIDAR_SDK_SRC} ${OPENLIDAR_SRC})
target_link_libraries (ol_lidar ${GPS_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
