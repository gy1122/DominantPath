//
//  main.cpp
//  DominantPath
//
//  Created by Ger Yang on 2016/6/30.
//  Copyright (c) 2016年 Ger Yang. All rights reserved.
//

#include <iostream>
#include "floorplan.h"

#include <cmath>

int main(int argc, const char * argv[]) {
    
    Floorplan flp;
    
    double wall_loss = 1.0;
    double angle_loss = 0.1;
    double lambda = 100.0;
    
    int size_x = 10;
    int size_y = 10;
    
    flp.genRandomFloorplan(size_x, size_y, wall_loss);
    //flp.printFloorplan();
    
    std::cerr << "Floorplan generated." << std::endl;
    
    Point pts[2];
    pts[0].x = 2.5;
    pts[0].y = 1.5;
    pts[1].x = 8.3;
    pts[1].y = 6.3;
    
    // Floorplan *, # of measurement points, measurement points
    DominantPath dmp(&flp, 2, pts, angle_loss);
    dmp.generateG2();
    
    dmp.Dijkstra(lambda, 0, 1);
    
    // id of the inspected point, scale, shift
    //dmp.printG2(39, 50, 5);
    dmp.printDijkstra(0, 1, 50, 5, lambda);
    return 0;
}
