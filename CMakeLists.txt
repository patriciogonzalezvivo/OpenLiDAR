cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lScanner)

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

# CELESTRON DRIVER
set(CELESTRON_DRIVER_PATH "include/celestron")
FILE(GLOB CELESTRON_DRIVER_SRC
    "${CELESTRON_DRIVER_PATH}/*.cpp"
)
include_directories(
    ${CELESTRON_DRIVER_PATH}
)

# RPLIDAR DRIVER
set(RPLIDAR_SDK_PATH "include/rplidar_sdk")
FILE(GLOB RPLIDAR_SDK_SRC 
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
)
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
)

add_executable (lScanner "src/main.cpp" "src/Scanner.cpp" ${RPLIDAR_SDK_SRC} ${CELESTRON_DRIVER_SRC})
target_link_libraries (lScanner ${PCL_LIBRARIES})
