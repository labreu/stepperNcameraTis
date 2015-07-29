project (microscan)

cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED )

add_executable(opencv-test  main.cpp v4ldevice.cpp)

target_link_libraries(opencv-test ${OpenCV_LIBS} -lv4l2 wiringPi)

