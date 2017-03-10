//
//  floorplan.h
//  DominantPath
//
//  Created by Ger Yang on 2016/6/30.
//  Copyright (c) 2016年 Ger Yang. All rights reserved.
//

#ifndef __DominantPath__floorplan__
#define __DominantPath__floorplan__

#include <vector>
#include <set>

#define TINY 1.0E-6
#define USE_OPEN_CV

#ifdef USE_OPEN_CV
#include <opencv2/highgui/highgui.hpp>
#endif  // USE_OPEN_CV

// ---------------------------------
//  Data structure for floorplan
// ---------------------------------

struct Corner;
struct Wall;

struct Corner {
    int    i;
    double x;
    double y;
};

struct Wall {

    inline double x1() const { return c1->x; }
    inline double y1() const { return c1->y; }
    inline double x2() const { return c2->x; }
    inline double y2() const { return c2->y; }

    Corner *c1;
    Corner *c2;
    double loss;
};


class Floorplan {

public:

    // Constructor
    Floorplan();
    ~Floorplan();

    void genRandomFloorplan(int x, int y, double wallloss, double angleloss,
                            double exterior_wallloss, unsigned seed = 0);

    void genOffice1(int nx, int ny, double wx, double wy, double hall,
                    double wallloss, double angleloss,
                    double exterior_wallloss);
    void genOffice2(int nx, int ny, double wx, double wy, double hall,
                    double wallloss, double angleloss,
                    double exterior_wallloss);
    void genOffice3(int nx, int ny, double wx, double wy, double hall,
                    double wallloss, double angleloss,
                    double exterior_wallloss);
    void genOffice4(int nx, int ny, double wx, double wy, double hall,
                    double wallloss, double angleloss,
                    double exterior_wallloss);

    inline int getNumCorners() const { return _nCorners; }
    inline Corner *getCornerPtr(int i) const { return &_corners[i]; }
    inline int getNumWalls() const { return _nWalls; }
    inline Wall *getWallPtr(int i) const { return &_walls[i]; }
    inline double getAngleLoss() const { return _angleLoss; }

    int save(const char *filename) const;
    int load(const char *filename);

    double getWidth() const;
    double getHeight() const;

private:

    double  _angleLoss; // angle loss per radian

    int     _nCorners;
    int     _nWalls;

    Corner  *_corners;
    Wall    *_walls;

};

#endif /* defined(__DominantPath__floorplan__) */
