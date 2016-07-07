cmake_minimum_required(VERSION 2.8)
project(DominantPath)
find_package(OpenCV REQUIRED)
add_executable(DominantPath main.cpp floorplan.cpp)
target_link_libraries(DominantPath ${OpenCV_LIBS})

