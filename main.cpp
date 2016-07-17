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
    
    // size_x, size_y, wall_loss
    flp.genRandomFloorplan(10, 10, 10.0);
    //flp.printFloorplan();
    
    std::cerr << "Floorplan generated." << std::endl;
    
    Point pts[2];
    pts[0].x = 2.5;
    pts[0].y = 1.5;
    pts[1].x = 7.3;
    pts[1].y = 8.3;
    
    // Floorplan *, # of measurement points, measurement points
    DominantPath dmp(&flp, 2, pts, 0.01);
    dmp.generateG2();
    
    dmp.Dijkstra(10.0, 0, 1);
    
    // id of the inspected point, scale, shift
    //dmp.printG2(39, 50, 5);
    dmp.printDijkstra(0.0, 0, 1, 50, 5);
    return 0;
}
