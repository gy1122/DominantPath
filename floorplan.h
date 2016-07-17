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
#include <map>

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


struct DijkstraPoint {
    DijkstraPoint(PointG2 *p_=0, int i_=0);
    
    PointG2 *p;
    int     i;
};

struct DijkstraLabel {
    DijkstraPoint   from;
    double          val;
    bool            visited;
};

DijkstraLabel &getDijkstraLabel(DijkstraPoint &p);
double getDijkstraVal(const DijkstraPoint &p);

struct WallG2 {

    Corner *to;
    
    // Derived variables
    double loss;
    double angle;
};

class PointG2 {
public:
    inline PointG2():dlabels(0) {}
    inline ~PointG2() { if (dlabels) delete [] dlabels; }
    
    inline double x() const { return ref->x; }
    inline double y() const { return ref->y; }
    virtual inline bool isCorner() const { return false; }
    
    int searchIdx(double angle) const;
    
    int        i;
    Point      *ref;
    
    // Links connect to this point
    //  - This is a sorted list
    //  - But instead, each link should keep which section it is so that we can compute the loss easily.
    //  - Also, this helps us know which side of the wall it is at.
    std::vector<EdgeG2> links;
    
    // Dijkstra labels
    DijkstraLabel* dlabels;
};

// We consider this to be directed.
struct EdgeG2 {
    
    PointG2     *target;
    int         target_i;
    int         source_i;
    
    // Derived variables
    double      loss;
    double      dist;
    double      angle;
    
    // with respect to the corner in v1.  If v1 is not a corner, then these two are not valid.
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
    
    DominantPath(Floorplan *flp, int nmpt, Point *mpts, double angleLoss);
    ~DominantPath();
    
    void generateG2();
    
    void Dijkstra(double lambda, int s, int t);
    
    void printG2(int p, double scale, double shift);
    void printDijkstra(double lambda, int s, int t, double scale, double shift);
    
private:
    
    // For generateG2():
    void initG2();
    void linkWalls();
    void createLinks(PointG2 *point);
    void createLinksForCorner(CornerG2 *corner);
    void createLinksForPoint(PointG2 *point);
    void mergeLinks(PointG2 *point, int nEdges, EdgeG2* tmpList, bool *tmpListKeep);
    void createLinksPostProc(PointG2 *point);
    
    //
    void initDijkstra();
    void resetDijkstra();
    
    double getLoss(const PointG2 *p, const EdgeG2 &edge) const;
    
    int findSection(const PointG2 *corner1, const CornerG2 *corner2, const EdgeG2 &inedge) const;
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
    
    
    double      _angleLoss;
};

#endif /* defined(__DominantPath__floorplan__) */
