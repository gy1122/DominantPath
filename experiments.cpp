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
#include <functional>
#include <random>
#include <set>
#define ARR_LIMIT 100

Random_st_pairs::Random_st_pairs(Floorplan *flp, double p):_flp(flp), _nTests(0), _p(p) {
    
}

Random_st_pairs::~Random_st_pairs() {
    
}

double Random_st_pairs::expected_loss(double r, int npaths, const Path* paths,
                                      const double* lambdas) const {
    double min_interval_loss = INFINITY;
    const double log_r = std::log(r);
    struct LossPoint {
        double lambda;
        double lambda_end;
        double loss;
        bool add;
        std::multiset<double>::const_iterator active_iterator;
        LossPoint(double _lambda, double _loss, bool _add,
                  double _lambda_end = 0.0) :
            lambda(_lambda), lambda_end(_lambda_end), loss(_loss), add(_add) {}
    };
    typedef std::function<bool(const LossPoint&, const LossPoint&)>
        LossPointCmp;
    typedef std::set<LossPoint,LossPointCmp> LossSet;
    LossSet losses([](const LossPoint& a, const LossPoint& b) {
            return (a.lambda < b.lambda ||
                    (a.lambda == b.lambda && a.add && !b.add) ||
                    (a.lambda == b.lambda && a.add == b.add &&
                     a.loss < b.loss));
        });
    std::multiset<double> active_losses;

    for (int i=0; i<npaths; ++i) {
        double loss = paths[i].L + _p * std::log(paths[i].D);
        if (i == 0 || i == npaths-1) {
            if (loss < min_interval_loss) {
                min_interval_loss = loss;
            }
            continue;
        }
        double log_lambda1 = std::log(lambdas[i]) / log_r;
        double log_lambda2 = std::log(lambdas[i+1]) / log_r;
        if (log_lambda2 - log_lambda1 >= 1.0) {
            if (loss < min_interval_loss) {
                min_interval_loss = loss;
            }
        } else {
            double log_lambda1_mod = std::fmod(log_lambda1, 1.0);
            double log_lambda2_mod = std::fmod(log_lambda2, 1.0);
            if (log_lambda1_mod < 0.0) log_lambda1_mod += 1.0;
            if (log_lambda2_mod < 0.0) log_lambda2_mod += 1.0;
            if (log_lambda1_mod <= log_lambda2_mod) {
                losses.emplace(log_lambda1_mod, loss, true,
                                         log_lambda2_mod);
            } else {
                losses.emplace(log_lambda1_mod, loss, true, 1.0);
                losses.emplace(0.0, loss, true, log_lambda2_mod);
            }
        }
    }
    double total_loss = 0.0;
    double prev_lambda = 0.0;
    active_losses.emplace(min_interval_loss);
    while (!losses.empty()) {
        LossPoint p = *losses.begin();
        losses.erase(losses.begin());
        total_loss += (p.lambda - prev_lambda) * *(active_losses.begin());
        prev_lambda = p.lambda;
        if (p.add) {
            p.active_iterator = active_losses.emplace(p.loss);
            p.add=false;
            p.lambda = p.lambda_end;
            losses.emplace(p);
        } else {
            active_losses.erase(p.active_iterator);
        }
    }

    total_loss += (1.0 - prev_lambda) * *(active_losses.begin());

    return total_loss;
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
        
        printf("Test %d (%f,%f) to (%f,%f) ", i, pts[0].x, pts[0].y, pts[1].x, pts[1].y);
        
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
        
        printf("minloss=%f@%d  ", min_loss, min_loss_id);

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
        
        for (int j = 0; j < npaths; j++) {
            //printf("%f ", lambdas[j]);
            printf("(%f,%f/%f)", paths[j].L, paths[j].D, lambdas[j]);
        }

        // output R
        if (min_loss_id == 0) {
            printf(" inf_first");
        } else if (min_loss_id == npaths-1) {
            printf (" inf_last");
        } else if (lambdas[min_loss_id] <= 0.0) {
            printf (" inf_zero");
        } else {
            printf (" %f", lambdas[min_loss_id+1]/lambdas[min_loss_id]);
        }

        for (double r : {1.2, 1.5, 2.0, 4.0, 10.0, 100.0, 1000.0}) {
            if (min_loss_id == 0 || min_loss_id == npaths-1 ||
                r < lambdas[min_loss_id+1] / lambdas[min_loss_id]) {
                printf (" 0.0");
            } else {
                printf (" %.9f", expected_loss(r, npaths, paths, lambdas) - min_loss);
            }
        }
        
        printf("\n");
        
        // Now R should be lambdas[min_loss_id+1]/lambdas[min_loss_id]
    }
}


int coverage(DominantPath &dmp) {
    
    int count = 0.0;
    double max_D_min = 0.0;
    double max_D_max = 0.0;
    
    // Find max_D_min
    count += dmp.Dijkstra_all_dest_corner(INFINITY);
    for (int i = 0; i < (int)dmp.getNumCorners(); i++) {
        double D_min = INFINITY;
        for (int j = 0; j < (int)dmp.getCorner(i)->links.size(); j++) {
            DijkstraPoint dp(dmp.getCorner(i), j);
            if (getDijkstraLabel(dp).visited) D_min = std::min(getDijkstraLabel(dp).dist, D_min);
        }
        max_D_min = std::max(D_min, max_D_min);
    }
    
    printf("max_t D_min = %f\n", max_D_min);
    
    // Find max_D_max
    count += dmp.Dijkstra_all_dest_corner(0);
    for (int i = 0; i < (int)dmp.getNumCorners(); i++) {
        double D_max = 0.0;
        for (int j = 0; j < (int)dmp.getCorner(i)->links.size(); j++) {
            DijkstraPoint dp(dmp.getCorner(i), j);
            if (getDijkstraLabel(dp).visited) D_max = std::max(getDijkstraLabel(dp).dist, D_max);
        }
        max_D_max = std::max(D_max, max_D_max);
    }
    
    printf("max_t D_max = %f\n", max_D_max);
    
    return count;
}
