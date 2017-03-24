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
//
// example heatmaps
// ./DominantPath -mode 3 -gm 10.0
// ./DominantPath -mode 3 -sx 12 -sy 7 -p0x 34.0 -p0y 15.0 -office 1 -gm 2.0
// ./DominantPath -mode 3 -sx 6 -sy 10 -p0x 25.0 -p0y 17.0 -office 2 -gm 4.0
// ./DominantPath -mode 3 -sx 10 -sy 10 -p0x 5.25 -p0y 12.5 -office 3 -gm 4.0
// ./DominantPath -mode 3 -sx 10 -sy 10 -p0x 5.25 -p0y 12.5 -office 4 -gm 4.0

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <cstring>
#include "dominantpath.h"
#include "experiments.h"
#include "floorplan.h"
#include "cputimer.h"

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
    const char *saveimage = 0;
    std::vector<int> pathset;

    bool display_paths = 1;
    bool label_floorplan = 1;
    double grid_measurements = 5.0;
    double step = 0.5;

    bool truncate_dijkstra = true;

    // quick hack for various computations
    // mode 0 == s-t breakpoint path computation
    // mode 1 == all destinations
    // mode 2 == approx all dest
    // mode 3 == heatmap
    // mode 4 == random s-t pair breakpoint evaluation
    // mode 5 == coverage
    // mode 6 == ratio_all_measurement
    int mode = 0;

    int num_experiments = 100;

    double p = 20.0/std::log(10.0);

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
        else if (strcmp(argv[argi], "-el") == 0) exterior_wall_loss = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-sd") == 0) seed = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-s") == 0) save = argv[++argi];
        else if (strcmp(argv[argi], "-l") == 0) load = argv[++argi];
        else if (strcmp(argv[argi], "-w") == 0) saveimage = argv[++argi];
        else if (strcmp(argv[argi], "-p0x") == 0) pts[0].x = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-p0y") == 0) pts[0].y = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-p1x") == 0) pts[1].x = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-p1y") == 0) pts[1].y = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-dp") == 0) display_paths = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-gm") == 0) grid_measurements = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-step") == 0) step = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-mode") == 0) mode = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-p") == 0) p = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-office") == 0) office = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-office_x") == 0) office_x = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-office_y") == 0) office_y = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-hall_width") == 0) hall_width = atof(argv[++argi]);
        else if (strcmp(argv[argi], "-n") == 0) num_experiments = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-path") == 0) pathset.push_back(atoi(argv[++argi]));
        else if (strcmp(argv[argi], "-truncate") == 0) truncate_dijkstra = atoi(argv[++argi]);
        else if (strcmp(argv[argi], "-label") == 0) label_floorplan = atoi(argv[++argi]);

        argi++;
    }

    if (load) {
        double start_time = util::cpu_timer();
        flp.load(load);
        std::cerr << "Floorplan loaded in " << (util::cpu_timer() - start_time)
                  << " cpu seconds." << std::endl;
    } else {
        double start_time = util::cpu_timer();
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
        std::cerr << "Floorplan generated in " << (util::cpu_timer() - start_time)
                  << " cpu seconds." << std::endl;
    }

    std::cerr << "Floorplan contains " << flp.getNumCorners() << " corners and "
              << flp.getNumWalls() << " walls." << std::endl;

    if (save) {
        double start_time = util::cpu_timer();
        flp.save(save);
        std::cerr << "Floorplan saved in " << (util::cpu_timer() - start_time)
                  << " cpu seconds." << std::endl;
    }

    int npts = 2;
    if (mode == 1 || mode == 2) npts = 3;

    if (mode == 3) label_floorplan = false;
    DominantPath dmp(&flp, npts, pts, label_floorplan);

    if (mode != 3 && mode != 4 && mode != 6) {
        double geng2_start = util::cpu_timer();
        dmp.generateG2();
        std::cerr << "G2 generated " << dmp.numG2Points() << " points, "
                  << dmp.numG2Links() << " links in "
                  << (util::cpu_timer() - geng2_start) << " cpu seconds."
                  << std::endl;
    }

    Path *paths = new Path[limit];
    int npaths = 2;
    if (mode == 0) {
        double break_start = util::cpu_timer();
        int count = dmp.BreakPoints(0, 1, limit, paths, npaths);
        std::cerr << count << " relaxations performed in "
                  << (util::cpu_timer() - break_start) << " cpu seconds."
                  << std::endl;
        if (display_paths) {
            dmp.printPaths(npaths, paths, p, saveimage, pathset);
        }
        for (int i = 0; i < npaths; i++) {
            printf("%f\n", (paths[i].L+p*std::log(paths[i].D)));
        }
    } else if (mode == 1) {
        double break_start = util::cpu_timer();
        dmp.Dijkstra_all_dest(100, paths);
        std::cerr << "Dijkstra_all_dest finished in "
                  << (util::cpu_timer() - break_start) << " cpu seconds."
                  << std::endl;
    } else if (mode == 2) {
        double break_start = util::cpu_timer();
        dmp.Approx_all_dest(p, step, paths, truncate_dijkstra);
        std::cerr << "Approx_all_dest finished in "
                  << (util::cpu_timer() - break_start) << " cpu seconds."
                  << std::endl;
    } else if (mode == 3) {
        dmp.heatmap(p, step, pts[0].x, pts[0].y,
                    flp.getWidth(), flp.getHeight(),
                    grid_measurements, saveimage, truncate_dijkstra);
    } else if (mode == 4) { // for testing
        Random_st_pairs rsp(&flp, p);
        rsp.size_x = flp.getWidth();
        rsp.size_y = flp.getHeight();
        rsp.test(num_experiments);
    } else if (mode == 5) {
        double break_start = util::cpu_timer();
        coverage(dmp);
        std::cerr << "coverage finished in "
        << (util::cpu_timer() - break_start) << " cpu seconds."
        << std::endl;
    } else if (mode == 6) {
        dmp.ratio_all_measurement(step, pts[0].x, pts[0].y,
                                  flp.getWidth(), flp.getHeight(),
                                  grid_measurements, false, false, true);
    }
    
    delete [] paths;

    return 0;
}
