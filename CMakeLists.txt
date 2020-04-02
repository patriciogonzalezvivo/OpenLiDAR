cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(3dscan)

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
include_directories(${GPS_INCLUDE_DIRS})
link_directories(${GPS_LIBRARY_DIRS})
add_definitions(${GPS_DEFINITIONS})

# Threads
find_package (Threads)

# OPENLIDAR
set(OPENLIDAR_PATH "libs/OpenLiDAR")
FILE(GLOB OPENLIDAR_SRC
    "${OPENLIDAR_PATH}/*.cpp"
    "${OPENLIDAR_PATH}/*/*.cpp"
)
include_directories(
    ${OPENLIDAR_PATH}
)

add_definitions(-DDEBUG_USING_SIMULATE_DATA)
add_executable (3dscan "apps/3dscan/main.cpp" ${RPLIDAR_SDK_SRC} ${OPENLIDAR_SRC} )
target_link_libraries (3dscan ${PCL_LIBRARIES} ${GPS_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
