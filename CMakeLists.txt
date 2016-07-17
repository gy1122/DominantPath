cmake_minimum_required(VERSION 2.8)
project(DominantPath)
find_package(OpenCV REQUIRED)
add_executable(DominantPath main.cpp floorplan.cpp dijkstra.cpp gen_g2.cpp)
target_link_libraries(DominantPath ${OpenCV_LIBS})

