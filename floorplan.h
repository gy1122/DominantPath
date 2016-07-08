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
    
    void genRandomFloorplan(int x, int y, double loss);
    
    void printFloorplan();
    
    inline int getNumCorners() const { return _nCorners; }
    inline Corner *getCornerPtr(int i) const { return &_corners[i]; }
    inline int getNumWalls() const { return _nWalls; }
    inline Wall *getWallPtr(int i) const { return &_walls[i]; }
    
private:
    
    int     _nCorners;
    int     _nWalls;
    
    Corner  *_corners;
    Wall    *_walls;
    
};


typedef Corner Point;

struct WallG2;
struct EdgeG2;
class PointG2;
class CornerG2;

struct WallG2 {

    Corner *to;
    double loss;
    
    // Let's keep these for walls
    // double dx;
    // double dy;
    double angle;
};

class PointG2 {
public:
    inline double x() const { return ref->x; }
    inline double y() const { return ref->y; }
    virtual inline bool isCorner() const { return false; }
    
    int        i;
    Point      *ref;
    
    // Links connect to this point
    //  - Different to the 'sides' list kept in CornerG2, this list does not have to be sorted.
    //  - But instead, each link should keep which section it is so that we can compute the loss easily.
    //  - Also, this helps us know which side of the wall it is at.
    std::vector<EdgeG2> links;
};

// We consider this to be directed.
struct EdgeG2 {
    
    PointG2     *to;
    
    double      loss;
    
    // Let's keep this for now
    double      dist;
    double      angle;
    
    // with respect to the corner in v1.  If v1 is not a corner, then this is not valid.
    int         section;
    
    int         isAlongWall;
};

class CornerG2 : public PointG2 {
public:
    virtual inline bool isCorner() const { return true; }
    
    void insertSide(WallG2 wall);
    
    // Walls connect to this corner
    //  - When creating this list, make sure to reorder v1 and v2 so that v1 is this corner.
    //  - Also, make sure this list is sorted by the incident angle.
    std::vector<WallG2> walls;
};


class DominantPath {
    
public:
    
    DominantPath(Floorplan *flp, int nmpt, Point *mpts);
    ~DominantPath();
    
    void generateG2();
    
    void printG2(int p, double scale, double shift);
    
private:
    
    double getLoss(const PointG2 *p, const EdgeG2 &edge) const;
    
    int findSection(const CornerG2 *corner1, const CornerG2 *corner2, const EdgeG2 &inedge) const;
    double cornerLoss(const CornerG2 *corner, int insec, int outsec) const;
    
    double _dx(const PointG2 *p, const EdgeG2 &edge) const;
    double _dy(const PointG2 *p, const EdgeG2 &edge) const;
    
    Floorplan   *_flp;
    
    // Measurement points
    int         _nmPoints;
    Point       *_mPoints;
    
    // We keep these as two separate lists
    int         _nG2Points;
    int         _nG2Corners;
    PointG2     *_G2Points;
    CornerG2    *_G2Corners;
    
    // Also create a combined list for references.  Not sure if this will be used later
    int         _nG2totPoints;
    PointG2     **_G2totPoints;
    
};

#endif /* defined(__DominantPath__floorplan__) */
