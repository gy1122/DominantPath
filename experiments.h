//
//  experiments.h
//  DominantPath
//
//  Created by Ger Yang on 2017/3/6.
//  Copyright (c) 2017年 Ger Yang. All rights reserved.
//

#ifndef __DominantPath__experiments__
#define __DominantPath__experiments__

#include "floorplan.h"

class Random_st_pairs {
public:
    Random_st_pairs(Floorplan *flp, double p);
    ~Random_st_pairs();
    
    void test(int nTests, unsigned seed = 123);

    double expected_loss(double r, int npaths, const Path* paths,
                         const double* lambdas) const;

    double         size_x;
    double         size_y;
    
private:
    Floorplan   *_flp;
    int         _nTests;
    double      _p;
};

int coverage(DominantPath &dmp);

#endif /* defined(__DominantPath__experiments__) */
