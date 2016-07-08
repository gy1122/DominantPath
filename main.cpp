//
//  main.cpp
//  DominantPath
//
//  Created by Ger Yang on 2016/6/30.
//  Copyright (c) 2016年 Ger Yang. All rights reserved.
//

#include <iostream>
#include "floorplan.h"

int main(int argc, const char * argv[]) {
    
    Floorplan flp;
    
    // size_x, size_y, wall_loss
    flp.genRandomFloorplan(10, 10, 0.1);
    //flp.printFloorplan();
    
    std::cerr << "Floorplan generated." << std::endl;
    
    Point pts[2];
    pts[0].x = 2.5;
    pts[0].y = 2;
    pts[1].x = 3.3;
    pts[1].y = 3.3;
    
    // Floorplan *, # of measurement points, measurement points
    DominantPath dmp(&flp, 2, pts);
    dmp.generateG2();
    
    // id of the inspected point, scale, shift
    dmp.printG2(50, 50, 5);
    return 0;
}
