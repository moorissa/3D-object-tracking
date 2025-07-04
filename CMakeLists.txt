cmake_minimum_required(VERSION 3.10)

# Set C++ standard to 17 (required for PCL 1.15)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")
message(STATUS "Final compile flags: ${CMAKE_CXX_FLAGS}")

project(camera_fusion)
if(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/usr/include")
endif()
string(REPLACE "-ffloat-store" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

find_package(OpenCV 4.1 REQUIRED)

# Add Boost package finding
find_package(Boost REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Include Boost headers
include_directories(${Boost_INCLUDE_DIRS})

# Executable for create matrix exercise
add_executable (
    3D_object_tracking
    src/camFusion_Student.cpp
    src/FinalProject_Camera.cpp
    src/lidarData.cpp 
    src/matching2D_Student.cpp
    src/objectDetection2D.cpp
)

# Link both OpenCV and Boost libraries
target_link_libraries (3D_object_tracking ${OpenCV_LIBRARIES})
