cmake_minimum_required(VERSION 2.8)
project( ColorTest )
find_package( OpenCV REQUIRED )
add_executable( ColorTest color.cpp )
target_link_libraries( ColorTest ${OpenCV_LIBS} )
