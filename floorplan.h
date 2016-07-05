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

struct Corner;
struct Wall;

struct Corner {
    double x;
    double y;
};

struct Wall {
    Corner *c1;
    Corner *c2;
    double loss;
};


class Floorplan {
    
public:
    
    // Constructor
    Floorplan();
    ~Floorplan();
    
    void genRandomFloorplan(int x, int y, double loss);
    
    void printFloorplan();
    
    int getNumCorners() const;
    Corner *getCornerPtr(int i) const;
    int getNumWalls() const;
    Wall *getWallPtr(int i) const;
    
private:
    
    int     _nCorners;
    int     _nWalls;
    
    Corner  *_corners;
    Wall    *_walls;
    
};


typedef Corner Point;
struct EdgeG1;
struct PointG1;


struct EdgeG1 {
    PointG1     *dest;
    double      angle;
    double      dist;
    double      loss;
};

struct PointG1 {
    Point               *ref;
    std::vector<EdgeG1> links;
    bool                isCorner;
};


class DominantPath {
    
public:
    
    DominantPath(Floorplan *flp, int nmpt, Point *mpts);
    ~DominantPath();
    
    void generateG1();
    
    void printG1();
    
private:
    
    Floorplan   *_flp;
    
    // measurement points
    int     _nmPoints;
    Point   *_mPoints;
    
    //
    int     _nG1Points;
    PointG1 *_G1Points;
    
};

#endif /* defined(__DominantPath__floorplan__) */
