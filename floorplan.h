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

// -------------------------------------
//  Data structure for G2 (declaration)
// -------------------------------------

typedef Corner Point;

struct WallG2;
struct EdgeG2;
class PointG2;
class CornerG2;

// ---------------------------------
//  Data structure for Dijkstra
// ---------------------------------

struct DijkstraPoint {

    // This struct works as the pointer to a vertex in G2.
    // If p is a corner, then i denotes the index of the socket in that corner.
    // If p is a measurement point, then i should always be zero.
    // ---------------------------------------------

    explicit DijkstraPoint(PointG2 *p_=0, int i_=0);

    PointG2 *p;
    int     i;
};

struct DijkstraLabel {

    // We keep this struct for each socket in G2
    // Each time we run the Dijkstra algorithm, we should call
    // DominantPath::resetDijkstra() to reset these labels.
    // ---------------------------------------------

    DijkstraPoint   from;
    double          val;
    double          dist;
    double          radar;
    bool            visited;

    double          l_e;
    double          d_e;
};

struct Path {

    inline void reset() {
        v.clear();
        L = 0.0;
        D = 0.0;
    }

    std::vector<DijkstraPoint> v;
    double L;
    double D;
};

// Priorty queue
// So far, the priority queue is implemented with std::set, where in most environment this should be RB-tree.
// std::set allows us to implement decrease_key more easily.
// -------------------------------------
class Priority_Queue {

public:
    struct mycmp_dijk_point {
        bool operator()(const DijkstraPoint &p1, const DijkstraPoint &p2) const;
    };

    typedef std::set<DijkstraPoint, mycmp_dijk_point> my_set;

private:
    my_set _queue;

public:
    inline void add(DijkstraPoint p) { _queue.insert(p); }
    inline my_set::size_type size() const { return _queue.size(); }

    DijkstraPoint extract_min();
    void decrease_key(DijkstraPoint &p, double newval);

    inline my_set::iterator begin() { return _queue.begin(); }

};

// Quick access functions:
DijkstraLabel &getDijkstraLabel(DijkstraPoint &p);
double getDijkstraVal(const DijkstraPoint &p);
double getDijkstraRadar(const DijkstraPoint &p);


// -------------------------------------
//  Data structure for G2 (declaration)
// -------------------------------------

struct WallG2 {

    // This data structure is converted from class Wall.
    // Each corner should keep a sorted list of this type.
    // ---------------------------------------------

    Corner *to;

    // Derived variables
    double loss;
    double angle;
};

class PointG2 {

    //
    // ---------------------------------------------

public:
    inline PointG2():ref(0),dlabels(0) {}
    inline ~PointG2() { if (dlabels) delete [] dlabels; }

    inline double x() const { return ref->x; }
    inline double y() const { return ref->y; }
    virtual inline bool isCorner() const { return false; }

    inline int eincr(int i) const { return (i+1) % links.size(); }
    inline int edecr(int i) const { return (i + links.size() -1) % (int)links.size(); }

    // is j between i and k in the cw order?
    inline bool between(bool cw, int i, int j, int k) const {
      if (cw) {
        return ((i <= j && j < k) || (k <= i && i <= j) || (j < k && k <= i));
      } else {
        return ((i >= j && j > k) || (k >= i && i >= j) || (j > k && k >= i));
      }
    }

    // This function searches the index of the output slot with the most similar angle
    int searchIdx(double angle) const;

    // i is the index of the point in DominantPath::_nG2totPoints
    int        i;
    Point      *ref;

    // Links connect to this point
    //  - This is a sorted list
    //  - Each link should keep which section it is so that we can compute the loss easily.
    std::vector<EdgeG2> links;

    // Dijkstra labels, this is a list with the same length as links.
    DijkstraLabel* dlabels;
};

struct EdgeG2 {

    // We consider each edge in G2 as directed.
    //
    // When constructing an instance of this struct,
    // it is guaranteed that an identical EdgeG2 instance with reversed direction
    // is also constructed.
    //
    // target_i : the index of the socket in the target node
    // source_i : the index of the socket in the source node
    // ---------------------------------------------

    PointG2     *target;
    int         target_i;
    int         source_i;

    // Derived variables
    double      loss;
    double      dist;
    double      angle;

    // with respect to the corner in tail.  If the tail is not a corner, then these two are not valid.
    int         section;
    int         isAlongWall;
};

class CornerG2 : public PointG2 {

    //
    // ---------------------------------------------

public:
    virtual inline bool isCorner() const { return true; }

    void insertSide(WallG2 wall);

    inline int wincr(int i) const { return (i+1) % walls.size(); }
    inline int wdecr(int i) const { return (i + sections() -1) % sections(); }
    inline int sections() const { return (int)walls.size(); }

    // Walls connect to this corner
    //  - When creating this list, make sure to reorder v1 and v2 so that v1 is this corner.
    //  - Also, make sure this list is sorted by the incident angle.
    //  ! You must use insertSide function to add a wall to this list.
    std::vector<WallG2> walls;

    // The index into links of the first link in each section
    std::vector<int> section_start;
};

#ifdef USE_OPEN_CV

namespace cv {
    class Mat;
}

#endif

// -------------------------------------
//  Main class
// -------------------------------------

class DominantPath {

    //
    // ---------------------------------------------

public:

    // Constructor:
    //  - flp: pointer to a Floorplan instance
    //  - nmpt: number of measurement points
    //  - mpts: a list of the measurement points
    // ---------------------------------------------
    DominantPath(Floorplan *flp, int nmpt, Point *mpts);
    ~DominantPath();

    // This function generates G2 from Floorplan *_flp
    void generateG2();

    // Stats about G2
    int numG2Points() const;
    int numG2Corners() const;
    int numG2MeasurementPoints() const;
    int numG2Links() const;

    // This function performs Dijkstra algorithm to find the shortest path from s to t
    // ---------------------------------------------
    // Inputs:
    //   - lambda: parameter for mixing two weights.  The weight will be L + labmda*D
    //   - s: index of the source measurement point
    //   - t: index of the destination measurement point
    //   - path: reference to the variable for storing the returned path
    // ---------------------------------------------
    // Output:
    //   - number of relaxations
    // ---------------------------------------------
    int Dijkstra(double lambda, int s, int t, Path &path);
    
    // This function performs Dijkstra algorithm to find the shortest path from 0 to all dests
    // ---------------------------------------------
    // Inputs:
    //   - lambda: parameter for mixing two weights.  The weight will be L + labmda*D
    //   - paths: reference to the array for storing the returned path
    // ---------------------------------------------
    // Output:
    //   - number of relaxations
    // ---------------------------------------------
    int Dijkstra_all_dest(double lambda, Path* &paths);
    
    // This function performs Dijkstra algorithm to find the shortest path from 0 to all dests and corners
    // ---------------------------------------------
    // Inputs:
    //   - lambda: parameter for mixing two weights.  The weight will be L + labmda*D
    //   - dists: distances for each dest and corner
    // ---------------------------------------------
    // Output:
    //   - number of relaxations
    // ---------------------------------------------
    int Dijkstra_all_dest_corner(double lambda);

    // This function finds all breakpoints for s-t paths
    // ---------------------------------------------
    // Input:
    //   - s: index of the source measurement point
    //   - t: index of the destination measurement point
    //   - limit: capacity of the list 'paths'
    //   - paths: a list of paths for storing the output paths.
    //   - npaths: reference the the variable for storing the number of breakpoints.
    // Output:
    //   - number of Dijkstra relaxations
    int BreakPoints(int s, int t, int limit, Path *paths, int &npaths);
    
    // This function approx the min loss path for all dests
    // ---------------------------------------------
    // Input:
    //   - p:
    //   - step:
    //   - paths: a list of paths for storing the output paths (the size should be equal to the number of measurement points - 1).
    // Output:
    //   - number of Dijkstra relaxations
    int Approx_all_dest(double p, double step, Path* &paths);

#ifdef USE_OPEN_CV

public:

    void printFloorplan();

    // This function prints part of G2
    //   - p: will plot the edges incident from point indexed p
    // ---------------------------------------------
    void printG2(int p);

    // This function prints the result of Dijkstra
    //   - npaths: number of paths to be printed
    //   - paths: the list for the paths to be printed
    // ---------------------------------------------
    void printPaths(int npaths, Path *paths, double p);
    // This function finds the heatmap
    void heatmap(double p, double step, double sx, double sy, double x, double y, double precision);

private:

    void makeFloorplanImage(cv::Mat &image, bool text = true);

#endif

private:

    // For generateG2():
    void initG2();
    void linkWalls();
    void createLinks(PointG2 *point);
    void createLinksForCorner(CornerG2 *corner);
    void createLinksForPoint(PointG2 *point);
    void mergeLinks(PointG2 *point, int nEdges, EdgeG2* tmpList);
    void createLinksPostProc(PointG2 *point);
    double getLoss(const PointG2 *p, const EdgeG2 &edge) const;

    // For Dijkstra:
    void initDijkstra();
    void resetDijkstra();
    int Dijkstra_cornerSwipe(Priority_Queue &Q, DijkstraPoint dpoint, int start_idx, double angle, double lambda, bool cw);
    int Dijkstra_sectionSwipe(Priority_Queue &Q, DijkstraPoint dpoint, int start_idx, int end_idx, double wall_loss, double angle, double lambda, bool cw);
    void relaxEdge(Priority_Queue &Q, DijkstraPoint dpoint, EdgeG2 edge, double lambda);
    void backTrack(double lambda, int s, int t, Path &path);

    double _Dijkstra_dist_constraint;

    // Helper functions:
    int findSection(const PointG2 *corner1, const CornerG2 *corner2, const EdgeG2 &inedge) const;
    double cornerLoss(const CornerG2 *corner, int insec, int outsec) const;

    double _dx(const PointG2 *p, const EdgeG2 &edge) const;
    double _dy(const PointG2 *p, const EdgeG2 &edge) const;
    double parametricWeight(double l, double d, double lambda);



    Floorplan   *_flp;

    // Measurement points
    int         _nmPoints;
    Point       *_mPoints;

    // We keep these as two separate lists
    int         _nG2Points;
    int         _nG2Corners;
    PointG2     *_G2Points;
    CornerG2    *_G2Corners;

    // Also create a combined list for references.
    int         _nG2totPoints;
    PointG2     **_G2totPoints;

public:
    double      fSizeX;
    double      fSizeY;
    double      fShift;
    double      fScale;
    
    inline CornerG2 *getCorner(int i) const { return &_G2Corners[i]; }
};

#endif /* defined(__DominantPath__floorplan__) */
