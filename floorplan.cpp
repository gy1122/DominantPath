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
#include <map>
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
    unsigned seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
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



void CornerG2::insertSide(WallG2 wall) {
    walls.push_back(wall);
    for (int i=(int)walls.size()-1; i>0; i--) {
        if (walls[i].angle < walls[i-1].angle)
            std::swap(walls[i], walls[i-1]);
    }
}

DominantPath::DominantPath(Floorplan *flp, int nmpt, Point *mpts):
    _flp(flp), _nmPoints(nmpt), _mPoints(mpts),
    _nG2Points(0), _nG2Corners(0), _G2Points(0), _G2Corners(0),
    _nG2totPoints(0), _G2totPoints(0) {
    
}

DominantPath::~DominantPath() {
    if (_G2Points) delete [] _G2Points;
}

bool mycmp(EdgeG2 edge1, EdgeG2 edge2) {
    return edge1.angle < edge2.angle;
}

void DominantPath::generateG2() {
    
    // Allocate the memory slots
    
    _nG2Points    = _nmPoints;
    _nG2Corners   = _flp->getNumCorners();
    _nG2totPoints = _nG2Points + _nG2Corners;
    
    _G2Points     = new PointG2[_nG2Points];
    _G2Corners    = new CornerG2[_nG2Corners];
    _G2totPoints  = new PointG2*[_nG2totPoints];
    
    
    // First create nodes for G2
    
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
    
    // Create links
    for (int i=0; i < _nG2totPoints; i++) {
        PointG2 *point = _G2totPoints[i];
        int nAdjWalls = 0;
        
        // This saves a temporary list of links
        //  - we'll sort this later
        EdgeG2 *tmpList = 0;
        int tmpIdx = 0;
        
        if (point->isCorner()) {
            CornerG2 *corner = reinterpret_cast<CornerG2 *>(point);
            
            // Create links based on walls
            nAdjWalls = (int)corner->walls.size();
            
            tmpList = new EdgeG2[_nG2totPoints + nAdjWalls];
            
            for (int j=0; j < nAdjWalls; j++) {
                WallG2 wall = corner->walls[j];
                
                // For each wall, create one link on each side
                EdgeG2 edge1, edge2;
                edge1.to = edge2.to = &_G2Corners[wall.to->i];
                edge1.section = j;
                edge2.section = (j+1) % nAdjWalls;
                edge1.isAlongWall = 1;
                edge2.isAlongWall = 2;
                
                double dx = _dx(point, edge1);
                double dy = _dy(point, edge1);
                
                //edge1.angle = edge2.angle = std::atan2(dy, dx);
                edge1.angle = edge2.angle = wall.angle;
                edge1.dist = edge2.dist = std::sqrt(dx*dx + dy*dy);
                
                // loss should be zero
                edge1.loss = edge2.loss = 0.0;
                
                tmpList[tmpIdx++] = edge1;
                tmpList[tmpIdx++] = edge2;
                
            }
        } else {
            tmpList = new EdgeG2[_nG2totPoints];
        }
        
        // Create links to other corners and measurement points
        for (int j=0; j < _nG2totPoints; j++) {
            if (j==i) continue;
            
            PointG2 *p2 = _G2totPoints[j];
            EdgeG2 edge;
            edge.to = p2;
            
            double dx = _dx(point, edge);
            double dy = _dy(point, edge);
            
            edge.angle = std::atan2(dy, dx);
            edge.dist = std::sqrt(dx*dx + dy*dy);
            edge.isAlongWall = 0;
            
            // add the link to the list
            tmpList[tmpIdx++] = edge;
            
            /* old implementation, this is slower
            // First check if this is collinear to existing links
            bool addthis = true;
            int replaceid = -1;
            for (int k=0; k < point->links.size(); k++) {
                
                // Check if the link already exists
                //  - when this link goes along a wall
                if (point->links[k].to == p2) {
                    addthis = false;
                    break;
                }
                
                if (std::abs(edge.angle - point->links[k].angle) < 1.0E-5) {
                    // Might need a more careful check, but we ignore it for now
                    
                    
                    if (edge.dist < point->links[k].dist) {
                        // If the longer line goes along the wall, we should never replace it.
                        if (point->links[k].isAlongWall) continue;
                        
                        replaceid = k;
                    }
                    
                    addthis = false;

                    break;
                }
            }
            
            if (addthis == false && replaceid == -1) continue;
            
            // find section
            if (point->isCorner()) {
                CornerG2 *corner = reinterpret_cast<CornerG2 *>(point);
                int sections = (int)corner->walls.size();
                int section = 0;
                for (int k=0; k < sections; k++) {
                    if (edge.angle < corner->walls[k].angle) break;
                    section++;
                }
                edge.section = section % sections;
            } else
                edge.section = 0; // not applicable
            
            // find the loss along the link
            edge.loss = getLoss(edge);
            
            // add the link
            if (addthis) point->links.push_back(edge);
            else if (replaceid >= 0) {
                point->links[replaceid] = edge;
            }
             */
        }
        
        // Sort the list and remove duplicated links
        std::sort(tmpList, tmpList + tmpIdx, mycmp);
        bool *tmpListKeep = new bool[tmpIdx];
        
        for (int j=0; j < tmpIdx; j++) tmpListKeep[j] = false;
        
        // j is the idx to keep
        for (int j=0, k=1;;k++) {
            // should we handle +pi/-pi?
            if (k == tmpIdx) {
                tmpListKeep[j] = true;
                break;
            }
            if (std::abs(tmpList[j].angle - tmpList[k].angle) > 1.0E-6) {
                tmpListKeep[j] = true;
                j=k;
            } else { // If two lines have the same angle
                // Intuitively, we should keep the line with smaller dist
                // However, in any way we should keep the lines along a wall
                
                // remove duplicated links
                if (tmpList[j].to == tmpList[k].to) {
                    if (tmpList[k].isAlongWall == 0) continue;
                    if (tmpList[j].isAlongWall == 0) {
                        j=k; continue;
                    }
                }
                
                if (tmpList[k].dist < tmpList[j].dist) {
                    // In this case, we should discard j, but if j is a line along a wall, we should keep it
                    if (tmpList[j].isAlongWall) {
                        tmpListKeep[j] = true;
                    }
                    j=k;
                } else {
                    // In this case, we are going to discard k, but again, if k is a line along a wall, we should keep it
                    if (tmpList[k].isAlongWall) {
                        tmpListKeep[k] = true;
                    }
                }
            }
        }
        
        for (int j=0; j < tmpIdx; j++) {
            if (tmpListKeep[j])
                point->links.push_back(tmpList[j]);
        }
        
        if (point->isCorner()) {
            int wallId = 0;
            
            // find which section it is in
            for (int j=0; j < point->links.size(); j++) {
                if (point->links[j].isAlongWall == 0) {
                    point->links[j].section = wallId % nAdjWalls;
                } else if (point->links[j].isAlongWall == 1) {
                    wallId++;
                }
            }
        }
        
        for (int j=0; j < point->links.size(); j++) {
            if (point->links[j].isAlongWall == 0) {
                point->links[j].loss = getLoss(point, point->links[j]);
            }
        }
        
        delete [] tmpListKeep;
        delete [] tmpList;
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
        
        if (p->ref == wall->c1 || edge.to->ref == wall->c1 || p->ref == wall->c2 || edge.to->ref == wall->c2) continue;
        
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

int DominantPath::findSection(const CornerG2 *corner1, const CornerG2 *corner2, const EdgeG2 &inedge) const {
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
    return (edge.to->x() - p->ref->x);
}

double DominantPath::_dy(const PointG2 *p, const EdgeG2 &edge) const {
    return (edge.to->y() - p->ref->y);
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
    std::sort(point->links.begin(), point->links.end(), mycmp);
    printf("   Angle, Sec,     Dist,  Loss\n");
    for (int i=0; i < point->links.size(); i++) {
        printf("%+.5f, %3d, % 3.5f, %.3e\n", point->links[i].angle, point->links[i].section, point->links[i].dist, point->links[i].loss);
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


