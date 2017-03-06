//
//  main.cpp
//  DominantPath
//
//  Created by Ger Yang on 2016/6/30.
//  Copyright (c) 2016年 Ger Yang. All rights reserved.
//

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <cstring>
#include "floorplan.h"

int main(int argc, const char * argv[]) {

    Floorplan flp;

    double wall_loss = 1.0;
    double angle_loss = 0.1;

    int size_x = 15;
    int size_y = 15;

    unsigned seed = 0;

    const char *save = 0;
    const char *load = 0;

    int display_paths = 1;

    // This program finds the paths from pts[0] to pts[1]
    Point pts[3];
    pts[0].x = 2.5;
    pts[0].y = 1.5;
    pts[1].x = 9.3;
    pts[1].y = 12.3;
    pts[2].x = 7.9;
    pts[2].y = 11.1;

    int limit = 50;

    int argi = 1;
    while (argi < argc - 1) {
        if (strcmp(argv[argi], "-sx") == 0) size_x = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-sy") == 0) size_y = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-wl") == 0) wall_loss = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-al") == 0) angle_loss = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-sd") == 0) seed = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-s") == 0) save = argv[++argi];
        else if (strcmp(argv[argi], "-l") == 0) load = argv[++argi];
        else if (strcmp(argv[argi], "-p0x") == 0) pts[0].x = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-p0y") == 0) pts[0].y = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-p1x") == 0) pts[1].x = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-p1y") == 0) pts[1].y = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-dp") == 0) display_paths = atoi(argv[++argi]);

        argi++;
    }

    if (load) {
        flp.load(load);
        std::cerr << "Floorplan loaded." << std::endl;
    } else {
        flp.genRandomFloorplan(size_x, size_y, wall_loss, angle_loss, seed);
        std::cerr << "Floorplan generated." << std::endl;
    }

    if (save) {
        flp.save(save);
        std::cerr << "Floorplan saved." << std::endl;
    }


    DominantPath dmp(&flp, 3, pts);
    //dmp.generateG2();

    Path *paths = new Path[limit];
    int npaths = 2;
    //int count = dmp.BreakPoints(0, 2, limit, paths, npaths);
    //printf("%d relaxation performed.\n", count);
    //dmp.Dijkstra_all_dest(100, paths);
    
    /*
    dmp.Approx_all_dest(5.0, 0.5, paths);
    if (display_paths) {
        dmp.printPaths(npaths, paths);
    }
    
    dmp.BreakPoints(0, 2, limit, paths, npaths);
    dmp.printPaths(npaths, paths);
    
    for (int i = 0; i < npaths; i++) {
        printf("%f\n", (paths[i].L+5.0*std::log(paths[i].D)));
    }
     */
    
    dmp.heatmap(5.0, 0.5, 45, 45, 15, 15, 5);
    
    delete [] paths;

    return 0;
}
