cmake_minimum_required(VERSION 2.8)
project(DominantPath)
find_package(OpenCV 3.1.0 REQUIRED)
add_executable(DominantPath main.cpp floorplan.cpp dijkstra.cpp gen_g2.cpp printimage.cpp heatmap.cpp experiments.cpp cputimer.cpp)
target_link_libraries(DominantPath ${OpenCV_LIBS})

if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -std=c++11")
endif()
