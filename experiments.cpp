//
//  experiments.cpp
//  DominantPath
//
//  Created by Ger Yang on 2017/3/6.
//  Copyright (c) 2017年 Ger Yang. All rights reserved.
//

#include "experiments.h"
#include <cmath>
#include <cassert>
#include <random>
#define ARR_LIMIT 100

Random_st_pairs::Random_st_pairs(Floorplan *flp, double p):_flp(flp), _nTests(0), _p(p) {
    
}

Random_st_pairs::~Random_st_pairs() {
    
}

void Random_st_pairs::test(int nTests, unsigned seed) {
    _nTests = nTests;
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    
    for (int i = 0; i < nTests; i++) {
        // Randomly generate test point
        Point pts[2];
        pts[0].x = distribution(generator) * size_x;
        pts[0].y = distribution(generator) * size_y;
        pts[1].x = distribution(generator) * size_x;
        pts[1].y = distribution(generator) * size_y;
        
        printf("Test (%f,%f) to (%f,%f)", pts[0].x, pts[0].y, pts[1].x, pts[1].y);
        
        // Find cvx hull for this s-t pair
        DominantPath dmp(_flp, 2, pts);
        dmp.generateG2();
        
        int npaths = 0;
        Path paths[ARR_LIMIT];
        dmp.BreakPoints(0, 1, ARR_LIMIT, paths, npaths);
        
        // Find the minimal loss path
        int min_loss_id = 0;
        double min_loss = INFINITY;
        
        for (int j = 0; j < npaths; j++) {
            double loss = paths[j].L + _p * std::log(paths[j].D);
            if (loss < min_loss) {
                min_loss = loss;
                min_loss_id = j;
            }
        }
        
        // Find the corresponding critical lambdas that switch from j-1 to j-th path
        double lambdas[ARR_LIMIT];
        lambdas[0] = 0.0; // min L
        
        if (npaths > 1)
            for (int j = 1; j < npaths; j++) {
                double dL = (paths[j].L - paths[j-1].L);
                double dD = (paths[j-1].D - paths[j].D);
                assert(dL >= 0 && dD >= 0);

                lambdas[j] = dL / dD;
            }
        
        // Now R should be lambdas[min_loss_id+1]/lambdas[min_loss_id]
    }
}