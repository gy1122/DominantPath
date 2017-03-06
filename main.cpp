//
//  main.cpp
//  DominantPath
//
//  Created by Ger Yang on 2016/6/30.
//  Copyright (c) 2016年 Ger Yang. All rights reserved.
//

// examples:
// ./DominantPath -office 1 -sx 10 -sy 6 -p0x 0.5 -p0y 0.5 -p1x 57.5 -p1y 57.5
// ./DominantPath -office 2 -sx 8 -sy 15 -p0x 1 -p0y 5 -p1x 57.5 -p1y 58.5
// ./DominantPath -office 3 -sx 12 -sy 12 -p0x 10 -p0y 10 -p1x 35.5 -p1y 44.5
// ./DominantPath -office 4 -sx 12 -sy 12 -p0x 2 -p0y 1 -p1x 20.5 -p1y 45.5
// ./DominantPath -sd 181 -p1x 8.5 -p1y 3.3

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <cstring>
#include <sys/time.h>
#include <sys/resource.h>
#include "floorplan.h"

double cpu_timer() {
    struct rusage usage;
    (void) getrusage(RUSAGE_SELF, &usage);
    return usage.ru_utime.tv_sec * 1.0 + usage.ru_utime.tv_usec / 1e6;
}

int main(int argc, const char * argv[]) {
    const double pi = std::abs(std::atan2(0,-1));
    Floorplan flp;

    double wall_loss = 2.0;                      // layered drywall
    double angle_loss = 5.0 / (pi/2);            // 5.0dB for 90 degrees
    double exterior_wall_loss = 15.0;            // thick concrete

    int size_x = 15;
    int size_y = 15;

    unsigned seed = 0;
    int office = 0;
    double office_x = 3.0;
    double office_y = 4.0;
    double hall_width = 2.0;

    const char *save = 0;
    const char *load = 0;

    int display_paths = 1;

    double logd_scale = 20.0/log(10.0);

    // This program finds the paths from pts[0] to pts[1]
    Point pts[2];
    pts[0].x = 2.5;
    pts[0].y = 1.5;
    pts[1].x = 9.3;
    pts[1].y = 12.3;

    int limit = 50;

    int argi = 1;
    while (argi < argc - 1) {
        if (strcmp(argv[argi], "-sx") == 0) size_x = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-sy") == 0) size_y = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-wl") == 0) wall_loss = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-al") == 0) angle_loss = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-el") == 0) exterior_wall_loss = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-sd") == 0) seed = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-s") == 0) save = argv[++argi];
        else if (strcmp(argv[argi], "-l") == 0) load = argv[++argi];
        else if (strcmp(argv[argi], "-p0x") == 0) pts[0].x = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-p0y") == 0) pts[0].y = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-p1x") == 0) pts[1].x = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-p1y") == 0) pts[1].y = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-dp") == 0) display_paths = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-scale") == 0) logd_scale = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-office") == 0) office = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-office_x") == 0) office_x = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-office_y") == 0) office_y = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-hall_width") == 0) hall_width = atof(argv[++argi]);

        argi++;
    }

    if (load) {
        double start_time = cpu_timer();
        flp.load(load);
        std::cerr << "Floorplan loaded in " << (cpu_timer() - start_time)
                  << " cpu seconds." << std::endl;
    } else {
        double start_time = cpu_timer();
        if (office == 1) {
            flp.genOffice1(size_x, size_y, office_x, office_y, hall_width,
                           wall_loss, angle_loss, exterior_wall_loss);
        } else if (office == 2) {
            flp.genOffice2(size_x, size_y, office_x, office_y, hall_width,
                           wall_loss, angle_loss, exterior_wall_loss);
        } else if (office == 3) {
            flp.genOffice3(size_x, size_y, office_x, office_y, hall_width,
                           wall_loss, angle_loss, exterior_wall_loss);
        } else if (office == 4) {
            flp.genOffice4(size_x, size_y, office_x, office_y, hall_width,
                           wall_loss, angle_loss, exterior_wall_loss);
        } else {
            flp.genRandomFloorplan(size_x, size_y, wall_loss, angle_loss,
                                   exterior_wall_loss, seed);
        }
        std::cerr << "Floorplan generated in " << (cpu_timer() - start_time)
                  << " cpu seconds." << std::endl;
    }

    std::cerr << "Floorplan contains " << flp.getNumCorners() << " corners and "
              << flp.getNumWalls() << " walls." << std::endl;

    if (save) {
        double start_time = cpu_timer();
        flp.save(save);
        std::cerr << "Floorplan saved in " << (cpu_timer() - start_time)
                  << " cpu seconds." << std::endl;
    }


    double geng2_start = cpu_timer();
    DominantPath dmp(&flp, 2, pts);
    dmp.generateG2();
    std::cerr << "G2 generated in " << (cpu_timer() - geng2_start)
              << " cpu seconds." << std::endl;

    Path *paths = new Path[limit];
    int npaths;
    double break_start = cpu_timer();
    int count = dmp.BreakPoints(0, 1, limit, paths, npaths);
    std::cerr << count << " relaxations performed in "
              << (cpu_timer() - break_start) << " cpu seconds." << std::endl;
    if (display_paths) {
        dmp.printPaths(npaths, paths, logd_scale);
    }

    delete [] paths;

    return 0;
}
