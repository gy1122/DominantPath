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
    flp.genRandomFloorplan(5, 5, 0.1);
    //flp.printFloorplan();
    
    std::cerr << "Floorplan generated." << std::endl;
    
    Point pts[2];
    pts[0].x = 2.5;
    pts[0].y = 2;
    pts[1].x = 3.3;
    pts[1].y = 3.3;
    DominantPath dmp(&flp, 2, pts);
    dmp.generateG2();
    dmp.printG2(16, 100, 5);
    return 0;
}