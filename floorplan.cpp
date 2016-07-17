//
//  floorplan.cpp
//  DominantPath
//
//  Created by Ger Yang on 2016/6/30.
//  Copyright (c) 2016年 Ger Yang. All rights reserved.
//

#include "floorplan.h"
#include <chrono>
#include <random>
#include <iostream>
#include <cmath>
#include <set>
#include <cassert>
#include <algorithm>
#include <cstdio>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define SHOW_DEBUG


Floorplan::Floorplan(): _nCorners(0), _nWalls(0), _corners(0), _walls(0) {
    
}

Floorplan::~Floorplan() {
    if (_corners)   delete [] _corners;
    if (_walls)     delete [] _walls;
}

void Floorplan::genRandomFloorplan(int x, int y, double loss) {
    
    // -- Form a random spanning tree
    int n_visited = 0;
    int  *visited = new int[x*y];
    int  *visited_list = new int[x*y];
    bool *vedges = new bool[x*y];
    bool *hedges = new bool[x*y];
    
    for (int i=0; i < x*y; i++) visited[i] = false;
    for (int i=0; i < x*y; i++) vedges[i] = true;
    for (int i=0; i < x*y; i++) hedges[i] = true;
    
    // Random number generator
    //unsigned seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
    unsigned seed = 0;
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    
    // Generate random edge weights
    double *wvedges = new double[x*y];
    double *whedges = new double[x*y];
    for (int i=0; i < x*y; i++) wvedges[i] = distribution(generator);
    for (int i=0; i < x*y; i++) whedges[i] = distribution(generator);
    
    
    // visit (0,0)
    n_visited++;
    visited_list[0] = 0;
    visited[0] = true;
    
    // Naive Prim's algorithm, without using priority queues
    const double INF = 1.0E9;
    while (n_visited < x*y) {
        
        double min_cost = INF;
        int min_node = -1;
        int min_dir = -1; // direction
        for (int i=0; i < n_visited; i++) {
            int j = visited_list[i];
            int cx = j % x;
            int cy = j / x;
            
            // check adjacent nodes of visited nodes
            if (cx>0 && !visited[j-1] && wvedges[j-1]<min_cost) {
                min_cost = wvedges[j-1];
                min_node = j;
                min_dir = 1;
            }
            if (cx<x-1 && !visited[j+1] && wvedges[j]<min_cost) {
                min_cost = wvedges[j];
                min_node = j;
                min_dir = 2;
            }
            if (cy>0 && !visited[j-x] && whedges[j-x]<min_cost) {
                min_cost = whedges[j-x];
                min_node = j;
                min_dir = 3;
            }
            if (cy<y-1 && !visited[j+x] && whedges[j]<min_cost) {
                min_cost = whedges[j];
                min_node = j;
                min_dir = 4;
            }
        }
        
        switch (min_dir) {
            case 1: visited[min_node-1] = true; vedges[min_node-1] = false; visited_list[n_visited] = min_node-1;
                break;
            case 2: visited[min_node+1] = true; vedges[min_node] = false; visited_list[n_visited] = min_node+1;
                break;
            case 3: visited[min_node-x] = true; hedges[min_node-x] = false; visited_list[n_visited] = min_node-x;
                break;
            case 4: visited[min_node+x] = true; hedges[min_node] = false; visited_list[n_visited] = min_node+x;
        }
        n_visited++;
        
    }
    
    
    // OK, now we have a random spanning tree.
    // Next, generate the floorplan graph:
    
    // -- Generate corners
    _nCorners = (x+1)*(y+1);
    _corners = new Corner[_nCorners];
    for (int i=0; i<(x+1)*(y+1); i++) {
        _corners[i].i = i;
        _corners[i].x = i % (x+1);
        _corners[i].y = i / (x+1);
    }
    
    // -- Generate walls
    //    first count the number of walls
    _nWalls = x+y;
    for (int i=0; i < y; i++)
        for (int j=0; j < x; j++)
            if (vedges[i*x+j]) _nWalls++;
    for (int i=0; i < y; i++)
        for (int j=0; j < x; j++)
            if (hedges[i*x+j]) _nWalls++;
    
    //    allocate the memory
    _walls = new Wall[_nWalls];
    
    int curptr = 0;
    
    for (int i=0; i < x; i++) {
        _walls[curptr].c1 = &_corners[i];
        _walls[curptr].c2 = &_corners[i+1];
        _walls[curptr].loss = loss;
        curptr++;
    }
    
    for (int i=0; i < y; i++) {
        _walls[curptr].c1 = &_corners[i*(x+1)];
        _walls[curptr].c2 = &_corners[(i+1)*(x+1)];
        _walls[curptr].loss = loss;
        curptr++;
    }
    
    for (int i=0; i < y; i++)
        for (int j=0; j < x; j++) {
            if (vedges[i*x+j]) {
                _walls[curptr].c1 = &_corners[i*(x+1)+j+1];
                _walls[curptr].c2 = &_corners[(i+1)*(x+1)+j+1];
                _walls[curptr].loss = loss;
                curptr++;
            }
            if (hedges[i*x+j]) {
                _walls[curptr].c1 = &_corners[(i+1)*(x+1)+j];
                _walls[curptr].c2 = &_corners[(i+1)*(x+1)+j+1];
                _walls[curptr].loss = loss;
                curptr++;
            }
        }
    
    
    delete [] visited;
    delete [] visited_list;
    delete [] vedges;
    delete [] hedges;
    delete [] wvedges;
    delete [] whedges;
}

// Check if the data structure is correct
void Floorplan::printFloorplan() {
    cv::Mat image(600,600,CV_8UC1,cv::Scalar(255));
    int scale = 40;
    int shift = 5;
    
    for (int i=0; i < _nCorners; i++) {
        Corner *corner = &_corners[i];
        cv::Point center(corner->x*scale+shift, corner->y*scale+shift);
        cv::circle(image, center, 3, cv::Scalar(0));
    }
    
    for (int i=0; i < _nWalls; i++) {
        Wall *wall = &_walls[i];
        cv::Point p1(wall->c1->x*scale+shift, wall->c1->y*scale+shift);
        cv::Point p2(wall->c2->x*scale+shift, wall->c2->y*scale+shift);
        cv::line(image, p1, p2, cv::Scalar(0), 2);
    }
    
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", image );
    cv::waitKey(0);
}

int PointG2::searchIdx(double angle) const {
    int min = 0;
    int max = (int)links.size();
    int chk;
    
    while (min < max-1) {
        chk = (min+max)/2;
        if (angle < links[chk].angle) max = chk;
        else min = chk;
    }
    return min;
}


void CornerG2::insertSide(WallG2 wall) {
    walls.push_back(wall);
    for (int i=(int)walls.size()-1; i>0; i--) {
        if (walls[i].angle < walls[i-1].angle)
            std::swap(walls[i], walls[i-1]);
    }
}

DominantPath::DominantPath(Floorplan *flp, int nmpt, Point *mpts, double angleLoss):
    _flp(flp), _nmPoints(nmpt), _mPoints(mpts),
    _nG2Points(0), _nG2Corners(0), _G2Points(0), _G2Corners(0),
    _nG2totPoints(0), _G2totPoints(0), _angleLoss(angleLoss) {
    
}

DominantPath::~DominantPath() {
    if (_G2Points) delete [] _G2Points;
}

bool mycmp_edge(EdgeG2 &edge1, EdgeG2 &edge2) {
    return (edge1.angle < edge2.angle) || (edge1.angle == edge2.angle && edge1.isAlongWall < edge2.isAlongWall);
}

void DominantPath::initG2() {
    
    // Allocate the memory slots
    
    _nG2Points    = _nmPoints;
    _nG2Corners   = _flp->getNumCorners();
    _nG2totPoints = _nG2Points + _nG2Corners;
    
    _G2Points     = new PointG2[_nG2Points];
    _G2Corners    = new CornerG2[_nG2Corners];
    _G2totPoints  = new PointG2*[_nG2totPoints];
    
    
    // Create nodes for G2
    
    int idx = 0;
    for (int i=0; i < _nG2Points; i++) {
        _G2Points[i].ref = &_mPoints[i];
        _G2totPoints[idx] = &_G2Points[i];
        _G2totPoints[idx]->i = idx;
        idx++;
    }
    for (int i=0; i < _nG2Corners; i++) {
        _G2Corners[i].ref = _flp->getCornerPtr(i);
        _G2totPoints[idx] = &_G2Corners[i];
        _G2totPoints[idx]->i = idx;
        idx++;
    }
}

void DominantPath::linkWalls() {
    
    // Link walls to corners
    
    for (int i=0; i < _flp->getNumWalls(); i++) {
        Wall *wptr = _flp->getWallPtr(i);
        
        // Create two walls for both directions
        WallG2 wall1, wall2;
        
        // wall1: c1 --> c2
        wall1.to = wptr->c2;
        wall1.loss = wptr->loss;
        
        // wall2: c2 --> c1
        wall2.to = wptr->c1;
        wall2.loss = wptr->loss;
        
        double dx = wptr->x2() - wptr->x1();
        double dy = wptr->y2() - wptr->y1();
        
        wall1.angle = std::atan2(dy, dx);
        wall2.angle = std::atan2(-dy, -dx);
        
        int idx1 = wptr->c1->i;
        int idx2 = wptr->c2->i;
        
        _G2Corners[idx1].insertSide(wall1);
        _G2Corners[idx2].insertSide(wall2);
    }
}

void DominantPath::createLinksForCorner(CornerG2 *corner) {

    // Create links based on walls
    int nAdjWalls = (int)corner->walls.size();
    int numLinks = _nG2totPoints + nAdjWalls*2 - 1;
    
    // This saves a temporary list of links
    //  - we'll sort this later
    EdgeG2 *tmpList = new EdgeG2[numLinks];
    bool *tmpListKeep = new bool[numLinks];
    
    int tmpIdx = 0;
    
    // Copy the links in the list to the tmp list, we will process these later.
    for (int j=0; j < corner->links.size(); j++) {
        tmpList[tmpIdx++] = corner->links[j];
    }
    
    for (int j=0; j < nAdjWalls; j++) {
        WallG2 wall = corner->walls[j];
        CornerG2 *target = &_G2Corners[wall.to->i];
        
        if (wall.to->i + _nmPoints < corner->i) continue;
        
        // For each wall, create one link on each side
        EdgeG2 edge1, edge2;
        
        edge1.target = edge2.target = target;
        
        edge1.section = j;
        edge2.section = (j+1) % nAdjWalls;
        edge1.isAlongWall = 1;
        edge2.isAlongWall = 2;
        
        double dx = _dx(corner, edge1);
        double dy = _dy(corner, edge1);
        
        edge1.angle = edge2.angle = wall.angle;
        edge1.dist = edge2.dist = std::sqrt(dx*dx + dy*dy);
        
        // loss should be zero
        edge1.loss = edge2.loss = 0.0;
        
        tmpList[tmpIdx++] = edge1;
        tmpList[tmpIdx++] = edge2;
    }
    
    // Create links to other corners and measurement points
    for (int j=0; j < _nG2totPoints; j++) {
        if (j <= corner->i) continue;
        
        PointG2 *p2 = _G2totPoints[j];
        EdgeG2 edge;
        edge.target = p2;
        
        double dx = _dx(corner, edge);
        double dy = _dy(corner, edge);
        
        edge.angle = std::atan2(dy, dx);
        edge.dist = std::sqrt(dx*dx + dy*dy);
        edge.isAlongWall = 0;
        
        // add the link to the list
        tmpList[tmpIdx++] = edge;
        
    }
    
    assert(tmpIdx <= numLinks);
    numLinks = tmpIdx;
    mergeLinks(corner, numLinks, tmpList, tmpListKeep);
    
    int wallId = 0;
    
    // find which section it is in
    for (int j=0; j < corner->links.size(); j++) {
        if (corner->links[j].isAlongWall == 0) {
            corner->links[j].section = wallId % nAdjWalls;
        } else if (corner->links[j].isAlongWall == 1) {
            wallId++;
        }
    }
    
    createLinksPostProc(corner);
    
    delete [] tmpList;
    delete [] tmpListKeep;
}

void DominantPath::createLinksForPoint(PointG2 *point) {
    
    int numLinks = _nG2totPoints - 1;
    
    // This saves a temporary list of links
    //  - we'll sort this later
    EdgeG2 *tmpList = new EdgeG2[numLinks];
    bool *tmpListKeep = new bool[numLinks];
    
    int tmpIdx = 0;
    
    // Copy the links in the list to the tmp list, we will process these later.
    for (int j=0; j < point->links.size(); j++) {
        tmpList[tmpIdx++] = point->links[j];
    }
    
    // Create links to other corners and measurement points
    for (int j=0; j < _nG2totPoints; j++) {
        if (j <= point->i) continue;
        
        PointG2 *p2 = _G2totPoints[j];
        EdgeG2 edge;
        edge.target = p2;
        
        double dx = _dx(point, edge);
        double dy = _dy(point, edge);
        
        edge.angle = std::atan2(dy, dx);
        edge.dist = std::sqrt(dx*dx + dy*dy);
        edge.isAlongWall = 0;
        
        // add the link to the list
        tmpList[tmpIdx++] = edge;
        
    }
    
    assert(tmpIdx <= numLinks);
    numLinks = tmpIdx;
    mergeLinks(point, numLinks, tmpList, tmpListKeep);
    
    createLinksPostProc(point);
    
    delete [] tmpList;
    delete [] tmpListKeep;
}

void DominantPath::mergeLinks(PointG2 *point, int nEdges, EdgeG2 *tmpList, bool *tmpListKeep) {
    
    // Sort the list and remove duplicated links
    std::sort(tmpList, tmpList + nEdges, mycmp_edge);
    
    for (int j=0; j < nEdges; j++) tmpListKeep[j] = false;
    
    // j is the idx to keep
    int first = -1;
    for (int j=0, k=1;;k++) {
        // should we handle +pi/-pi?
        if (k == nEdges) {
            //tmpListKeep[j] = true;
            if (tmpList[j].target == tmpList[first].target && j != first) {
                if (tmpList[j].isAlongWall) {
                    tmpListKeep[j] = true;
                    tmpListKeep[first] = false;
                }
            } else {
                tmpListKeep[j] = true;
            }
            break;
        }
        if (std::abs(tmpList[j].angle - tmpList[k].angle) > 1.0E-6) {
            tmpListKeep[j] = true;
            if (first < 0) first = j;
            j=k;
        } else { // If two lines have the same angle
            // Intuitively, we should keep the line with smaller dist
            // However, in any way we should keep the lines along a wall
            
            // remove duplicated links
            if (tmpList[j].target == tmpList[k].target) {
                if (tmpList[k].isAlongWall == 0) continue;
                if (tmpList[j].isAlongWall == 0) {
                    // don't keep j
                    j=k; continue;
                }
            }
            
            if (tmpList[k].dist < tmpList[j].dist) {
                // In this case, we should discard j
                if (tmpList[j].isAlongWall || tmpList[j].target->i < point->i) {
                    tmpListKeep[j] = true;
                    if (first < 0) first = j;
                }
                j=k;
            } else {
                // In this case, we should discard k
                if (tmpList[k].isAlongWall || tmpList[k].target->i < point->i) {
                    tmpListKeep[k] = true;
                }
            }
        }
    }
    
    point->links.clear();
    for (int j=0; j < nEdges; j++) {
        //if (tmpListKeep[j] || tmpList[j].isAlongWall || tmpList[j].target->i < point->i)
        if (tmpListKeep[j])
            point->links.push_back(tmpList[j]);
        
        if (tmpList[j].isAlongWall || tmpList[j].target->i < point->i)
            assert(tmpListKeep[j]);
    }

}

void DominantPath::createLinksPostProc(PointG2 *point) {
    
    for (int j=0; j < point->links.size(); j++) {
        PointG2 *target = point->links[j].target;
        
        // Find loss
        if (point->links[j].isAlongWall == 0 && target->i > point->i) {
            point->links[j].loss = getLoss(point, point->links[j]);
        }
        
        point->links[j].source_i = j;
        
        if (target->i < point->i) {
            // find the reverse link
            int target_i = point->links[j].target_i;
            // update the target's target_i
            target->links[target_i].target_i = j;
        }
    }
    
    // Create reverse links
    for (int j=0; j < point->links.size(); j++) {
        EdgeG2 edge = point->links[j];
        PointG2 *target = edge.target;
        
        if (target->i < point->i) continue;
        
        if (target->isCorner()) {
            CornerG2 *target_corner = reinterpret_cast<CornerG2 *>(target);
            int sec = findSection(point, target_corner, edge);
            // 1           2
            // -------------
            // 2           1
            if (edge.isAlongWall) {
                int wallId;
                if (edge.isAlongWall == 1) {
                    wallId = (sec + target_corner->walls.size() - 1) % (int)target_corner->walls.size();
                    edge.isAlongWall = 2; // revert this
                } else {
                    wallId = sec;
                    edge.isAlongWall = 1;
                }
                edge.target = point;
                edge.angle = target_corner->walls[wallId].angle;
            } else {
                // compute angle
                edge.target = point;
                edge.angle = atan2(_dy(target, edge), _dx(target, edge));
            }
            edge.section = sec;
            
        } else {
            edge.target = point;
            edge.angle = atan2(_dy(target, edge), _dx(target, edge));
            // Keep loss & dist
        }

        edge.target_i = j;
        target->links.push_back(edge);
    }
}
    

void DominantPath::createLinks(PointG2 *point) {
    
    if (point->isCorner()) createLinksForCorner(reinterpret_cast<CornerG2 *>(point));
    else createLinksForPoint(point);
    
}

void DominantPath::generateG2() {
    
    initG2();
    linkWalls();
    
    // Create links
    for (int i=0; i < _nG2totPoints; i++) {
        createLinks(_G2totPoints[i]);
    }

    initDijkstra();
}

void DominantPath::initDijkstra() {
    for (int i=0; i < _nG2totPoints; i++) {
        PointG2 *point = _G2totPoints[i];
        if (point->isCorner()) {
            point->dlabels = new DijkstraLabel[point->links.size()];
        } else {
            point->dlabels = new DijkstraLabel[1];
        }
    }
}

void DominantPath::resetDijkstra() {
    for (int i=0; i < _nG2totPoints; i++) {
        PointG2 *point = _G2totPoints[i];
        if (point->isCorner()) {
            for (int j=0; j < point->links.size(); j++) {
                point->dlabels[j].from.p = 0;
                point->dlabels[j].from.i = 0;
                point->dlabels[j].val = INFINITY;
                point->dlabels[j].visited = false;
            }
        } else {
            point->dlabels[0].from.p = 0;
            point->dlabels[0].from.i = 0;
            point->dlabels[0].val = INFINITY;
            point->dlabels[0].visited = false;
        }
    }
}

DijkstraPoint::DijkstraPoint(PointG2 *p_, int i_):p(p_),i(i_) {
    if (p && !p->isCorner()) i = 0;
}

DijkstraLabel &getDijkstraLabel(DijkstraPoint &p) {
    return p.p->dlabels[p.i];
}

double getDijkstraVal(const DijkstraPoint &p) {
    return p.p->dlabels[p.i].val;
}

class Priority_Queue {
    struct mycmp_dijk_point {
        bool operator()(const DijkstraPoint &p1, const DijkstraPoint &p2) const {
            double v1 = getDijkstraVal(p1);
            double v2 = getDijkstraVal(p2);
            return (v1 < v2) || (v1 == v2 && p1.p < p2.p) || (v1 == v2 && p1.p == p2.p && p1.i < p2.i);
        }
    };
    
private:
    typedef std::set<DijkstraPoint, mycmp_dijk_point> my_set;
    my_set _queue;
    
public:
    void add(DijkstraPoint p) { _queue.insert(p); }
    
    DijkstraPoint extract_min() {
        DijkstraPoint p = *(_queue.begin());
        _queue.erase(_queue.begin());
        return p;
    }
    
    void decrease_key(DijkstraPoint &p, double newval) {
        // If p is not been visited
        if (getDijkstraVal(p) == INFINITY) {
            getDijkstraLabel(p).val = newval;
            _queue.insert(p);
        } else {
            my_set::iterator it = _queue.find(p);
            assert(it != _queue.end());
            _queue.erase( it );
            getDijkstraLabel(p).val = newval;
            _queue.insert(p);
        }
    }
    
    size_t size() { return _queue.size(); }
  
};

void DominantPath::Dijkstra(double lambda, int s, int t) {
    resetDijkstra();
    
    Priority_Queue Q;
    
    DijkstraPoint source(_G2totPoints[s]);
    getDijkstraLabel(source).val = 0.0;
    Q.add(source);
    
    while (Q.size() > 0) {
        DijkstraPoint dpoint = Q.extract_min();
        PointG2 *point = dpoint.p;
        getDijkstraLabel(dpoint).visited = true;
        
        printf("%d %d %f\n", point->i, dpoint.i, getDijkstraVal(dpoint));
        
        if (point->i == t) break;
        
        if (point->isCorner()) {
            
            CornerG2 *corner = reinterpret_cast<CornerG2 *>(point);
            int size_list = (int)dpoint.p->links.size();
            
            double wall_loss;
            
            double pi = std::abs(std::atan2(0,-1));
            double in_angle = point->links[dpoint.i].angle;
            if (in_angle > 0) in_angle -= pi;
            else in_angle += pi;
            
            int start_idx = point->searchIdx(in_angle);
            
            // clockwise
            wall_loss = cornerLoss(corner, point->links[dpoint.i].section, point->links[(start_idx+1) % size_list].section);
            for (int i = (start_idx+1) % size_list; i != start_idx; i = (i+1) % size_list) {
                DijkstraPoint epoint(point, i);
                if (getDijkstraLabel(epoint).visited) break;
                
                double val = getDijkstraVal(epoint);
                
                double dAngle = (point->links[i].angle - in_angle);
                if (dAngle < 0) dAngle += 2.0*pi;
                assert(dAngle >= 0);
                
                double newval = getDijkstraVal(dpoint) + wall_loss + _angleLoss * dAngle;
                
                if (newval < val) {
                    Q.decrease_key(epoint, newval);
                    getDijkstraLabel(epoint).from = dpoint;
                    
                    DijkstraPoint epoint2(point->links[i].target, point->links[i].target_i);
                    double newval2 = newval + lambda * point->links[i].dist + point->links[i].loss;
                    
                    // check if this can be propagated one step forward
                    if (newval2 < getDijkstraVal(epoint2)) {
                        assert(!getDijkstraLabel(epoint2).visited);
                        Q.decrease_key(epoint2, newval2);
                        getDijkstraLabel(epoint2).from = epoint;
                    }
                } else break;
                
                if (point->links[i].isAlongWall == 1) wall_loss += corner->walls[point->links[i].section].loss;
            }
            
            // counterclockwise
            wall_loss = cornerLoss(corner, point->links[dpoint.i].section, point->links[start_idx].section);
            for (int i = start_idx % size_list; i != (start_idx + 1) % size_list; i = (i+size_list-1) % size_list) {
                DijkstraPoint epoint(point, i);
                if (getDijkstraLabel(epoint).visited) break;
                
                if (point->links[i].isAlongWall == 1) wall_loss += corner->walls[point->links[i].section].loss;
                
                double val = getDijkstraVal(epoint);
                double dAngle = (in_angle - point->links[i].angle);
                if (dAngle < 0) dAngle += 2.0*pi;
                assert(dAngle >= 0);
                
                double newval = getDijkstraVal(dpoint) + wall_loss + _angleLoss * dAngle;
                
                if (newval < val) {
                    Q.decrease_key(epoint, newval);
                    getDijkstraLabel(epoint).from = dpoint;
                    
                    DijkstraPoint epoint2(point->links[i].target, point->links[i].target_i);
                    double newval2 = newval + lambda * point->links[i].dist + point->links[i].loss;
                    
                    // check if this can be propagated one step forward
                    if (newval2 < getDijkstraVal(epoint2)) {
                        assert(!getDijkstraLabel(epoint2).visited);
                        Q.decrease_key(epoint2, newval2);
                        getDijkstraLabel(epoint2).from = epoint;
                    }
                } else break;
            }
            
        } else {
            for (int i=0; i < point->links.size(); i++) {
                EdgeG2 edge = point->links[i];
                DijkstraPoint epoint(edge.target, edge.target_i);
                
                if (getDijkstraLabel(epoint).visited) continue;
                
                // relax edge.target
                double val = getDijkstraVal(epoint);
                double newval = getDijkstraLabel(dpoint).val + (lambda * edge.dist + edge.loss);
                if (newval < val) {
                    Q.decrease_key(epoint, newval);
                    getDijkstraLabel(epoint).from = dpoint;
                }
            }
        }
        
    }
}

double DominantPath::getLoss(const PointG2 *p, const EdgeG2 &edge) const {
    double x = p->x();
    double y = p->y();
    double dx = _dx(p, edge);
    double dy = _dy(p, edge);

    double loss = 0.0;
    
    for (int i=0; i < _flp->getNumWalls(); i++) {
        Wall *wall = _flp->getWallPtr(i);
        
        if (p->ref == wall->c1 || edge.target->ref == wall->c1 || p->ref == wall->c2 || edge.target->ref == wall->c2) continue;
        
        double u = wall->c1->x;
        double v = wall->c1->y;
        double du = wall->c2->x - u;
        double dv = wall->c2->y - v;
        
        double z = dx*dv-dy*du;
        if (std::abs(z)>1.0E-6) {
            double r = ((y-v)*dx + (u-x)*dy)/z;
            double t = ((u-x)*dv + (y-v)*du)/z;
            if (r>0.0 && r<1.0 && t>0.0 && t<1.0) {
                loss += wall->loss;
            }
        }
    }
    
    return loss;
}

int DominantPath::findSection(const PointG2 *corner1, const CornerG2 *corner2, const EdgeG2 &inedge) const {
    // corner1 ---> corner2
    
    // First check if this comes along a wall
    if (inedge.isAlongWall) {
        // Find the corresponding wall
        int wallidx = -1;
        for (int i=0; i < corner2->walls.size(); i++) {
            if (corner2->walls[i].to == corner1->ref) {
                wallidx = i;
                break;
            }
        }
        assert(wallidx >= 0);
        /*
         1              2
         ----------------
         2              1
         */
        if (inedge.isAlongWall==1) return (wallidx+1)%(corner2->walls.size());
        else return wallidx;
    }
    
    // Else compute the angle
    double angle = atan2(-_dy(corner1,inedge), -_dx(corner1,inedge));
    int sections = (int)corner2->walls.size();
    int section = 0;
    for (int k=0; k < sections; k++) {
        if (angle < corner2->walls[k].angle) break;
        section++;
    }
    return section % sections;
}

double DominantPath::cornerLoss(const CornerG2 *corner, int insec, int outsec) const {
    double loss1 = 0.0;
    double loss2 = 0.0;
    int sections = (int)corner->walls.size();
    
    // clockwise
    for (int i=insec; i != outsec; i = (i+1) % sections) {
        loss1 += corner->walls[i].loss;
    }
    
    // counterclockwise
    for (int i=insec; i != outsec; i = (i+sections-1) % sections) {
        loss2 += corner->walls[(i+sections-1)%sections].loss;
    }
    
    std::cout << loss1 << "," << loss2 << std::endl;
    return MIN(loss1,loss2);
}

double DominantPath::_dx(const PointG2 *p, const EdgeG2 &edge) const {
    return (edge.target->x() - p->ref->x);
}

double DominantPath::_dy(const PointG2 *p, const EdgeG2 &edge) const {
    return (edge.target->y() - p->ref->y);
}

// Check if the data structure is correct
void DominantPath::printG2(int p, double scale, double shift) {
    cv::Mat image(800,800,CV_8UC3,cv::Scalar(255,255,255));
    
    for (int i=0; i < _nG2totPoints; i++) {
        PointG2 *point = _G2totPoints[i];
        cv::Point center(point->ref->x*scale+shift, point->ref->y*scale+shift);
        cv::circle(image, center, 3, cv::Scalar(0,0,0));
    }
    
    for (int i=0; i < _flp->getNumWalls(); i++) {
        Wall *wall = _flp->getWallPtr(i);
        cv::Point p1(wall->c1->x*scale+shift, wall->c1->y*scale+shift);
        cv::Point p2(wall->c2->x*scale+shift, wall->c2->y*scale+shift);
        cv::line(image, p1, p2, cv::Scalar(0,0,0), 2);
    }
    
    int j = p;
    PointG2 *point = _G2totPoints[j];
    cv::Point center(point->ref->x*scale+shift, point->ref->y*scale+shift);
    for (int i=0; i < point->links.size(); i++) {
        EdgeG2 edge = point->links[i];
        double x = point->x() + edge.dist * std::cos(edge.angle);
        double y = point->y() + edge.dist * std::sin(edge.angle);
        cv::Point p2(x*scale+shift, y*scale+shift);
        cv::line(image, center, p2, cv::Scalar(255,0,0));
    }
    
#ifdef SHOW_DEBUG
    // For debugging
    if (point->isCorner()) {
        CornerG2 *corner = reinterpret_cast<CornerG2 *>(point);
        printf("--Adjacent Walls--\n   Angle, Loss\n");
        for (int i=0; i < corner->walls.size(); i++) {
            printf("%+.5f, %.3f\n", corner->walls[i].angle, corner->walls[i].loss);
        }
    }
    printf("\nTotal links: %d\n", (int)point->links.size());
    //std::sort(point->links.begin(), point->links.end(), mycmp_edge);
    printf("   Angle, Sec,     Dist,  Loss\n");
    for (int i=0; i < point->links.size(); i++) {
        printf("%+.5f, %3d, % 3.5f, %.3e, %d\n", point->links[i].angle, point->links[i].section, point->links[i].dist, point->links[i].loss, point->links[i].isAlongWall);
    }
    
    /*
    if (point->isCorner()) {
        CornerG2 *corner = reinterpret_cast<CornerG2 *>(point);
        if (corner->walls.size()>1) std::cout << cornerLoss(corner, 1, 0) << std::endl;
    }
     */
#endif
    
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", image );
    cv::waitKey(0);
}

void DominantPath::printDijkstra(double lambda, int s, int t, double scale, double shift) {
    cv::Mat image(800,800,CV_8UC3,cv::Scalar(255,255,255));
    
    for (int i=0; i < _nG2totPoints; i++) {
        PointG2 *point = _G2totPoints[i];
        cv::Point center(point->ref->x*scale+shift, point->ref->y*scale+shift);
        cv::circle(image, center, 3, cv::Scalar(0,0,0));
    }
    
    for (int i=0; i < _flp->getNumWalls(); i++) {
        Wall *wall = _flp->getWallPtr(i);
        cv::Point p1(wall->c1->x*scale+shift, wall->c1->y*scale+shift);
        cv::Point p2(wall->c2->x*scale+shift, wall->c2->y*scale+shift);
        cv::line(image, p1, p2, cv::Scalar(0,0,0), 2);
    }
    
    // Back tracking
    DijkstraPoint dpoint(_G2totPoints[t], 0);
    while (dpoint.p->i != s) {
        DijkstraPoint nextPoint = getDijkstraLabel(dpoint).from;
        
        if (nextPoint.p != dpoint.p) {
            PointG2 *point1 = dpoint.p;
            PointG2 *point2 = nextPoint.p;
            cv::Point p1(point1->ref->x*scale+shift, point1->ref->y*scale+shift);
            cv::Point p2(point2->ref->x*scale+shift, point2->ref->y*scale+shift);
            cv::line(image, p1, p2, cv::Scalar(255,0,0));
        }
        
#ifdef SHOW_DEBUG
        printf("%d %d %f :: %d %d %f \n", dpoint.p->i, dpoint.i, getDijkstraVal(dpoint), nextPoint.p->i, nextPoint.i, getDijkstraVal(nextPoint));
#endif
        
        dpoint = nextPoint;
    }
    

    
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", image );
    cv::waitKey(0);
}


