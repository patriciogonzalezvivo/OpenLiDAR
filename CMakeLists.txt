cmake_minimum_required(VERSION 3.1)

if(${CMAKE_VERSION} VERSION_LESS 3.15)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
else()
    cmake_policy(VERSION 3.15)
endif()

project(OpenLiDAR)
# project(OpenLiDAR VERSION 0.1
#                   DESCRIPTION "Open 3D LiDAR Scanner "
#                   LANGUAGES CXX)

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    # Optionally set things like CMAKE_CXX_STANDARD, CMAKE_POSITION_INDEPENDENT_CODE here
    
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    # Let's ensure -std=c++xx instead of -std=g++xx
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Let's nicely support folders in IDE's
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

    # Docs only available if this is the main app
    find_package(Doxygen)
    if(Doxygen_FOUND)
	add_subdirectory(docs)
    else()
	message(STATUS "Doxygen not found, not building docs")
    endif()
endif()

# add_definitions(-DDEBUG_USING_SIMULATE_DATA)

# Compile dependencies
add_subdirectory(deps)

set(OPENLIDAR_SOURCE
    ${PROJECT_SOURCE_DIR}/src/OpenLiDAR.cpp
)

add_library(OpenLiDAR ${OPENLIDAR_SOURCE})

set_target_properties(OpenLiDAR PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
)

include_directories(
    ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(OpenLiDAR PUBLIC
    mary
)

# The executable code is here
add_subdirectory(apps)
