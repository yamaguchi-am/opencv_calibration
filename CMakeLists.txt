cmake_minimum_required(VERSION 2.6)

project(opencv_camera_calibration)
add_compile_options(-std=c++11 -Wall -g)

find_package(OpenCV 4 REQUIRED)
find_package(gflags)

if (NOT gflags_FOUND)
  add_definitions(-D gflags=google)
endif (NOT gflags_FOUND)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(calibrate
  calibrate.cc
)
target_link_libraries(calibrate
    ${OpenCV_LIBS}    
    gflags
)

add_executable(undistort
  undistort.cc
)
target_link_libraries(undistort
    ${OpenCV_LIBS}    
    gflags
)
