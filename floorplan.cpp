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
        idx++;
    }
    for (int i=0; i < _nG2Corners; i++) {
        _G2Corners[i].ref = _flp->getCornerPtr(i);
        _G2totPoints[idx] = &_G2Corners[i];
        idx++;
    }
    
    // Link walls to corners
    for (int i=0; i < _flp->getNumWalls(); i++) {
        Wall *wptr = _flp->getWallPtr(i);
        
        // Create two walls for both directions
        WallG2 wall1, wall2;
        
        wall1.c1 = wptr->c1;
        wall1.c2 = wptr->c2;
        wall1.loss = wptr->loss;
        
        wall2.c1 = wptr->c2;
        wall2.c2 = wptr->c1;
        wall2.loss = wptr->loss;
        
        wall1.dx = wall1.c2->x - wall1.c1->x;
        wall1.dy = wall1.c2->y - wall1.c1->y;
        wall1.angle = std::atan2(wall1.dy, wall1.dx);
        
        wall2.dx = wall2.c2->x - wall2.c1->x;
        wall2.dy = wall2.c2->y - wall2.c1->y;
        wall2.angle = std::atan2(wall2.dy, wall2.dx);
        
        int idx1 = wall1.c1->i;
        int idx2 = wall2.c1->i;
        
        _G2Corners[idx1].insertSide(wall1);
        _G2Corners[idx2].insertSide(wall2);
    }
    
    // Create links
    for (int i=0; i < _nG2totPoints; i++) {
        PointG2 *point = _G2totPoints[i];
        
        if (point->isCorner()) {
            CornerG2 *corner = reinterpret_cast<CornerG2 *>(point);
            
            // Create links based on walls
            int sections = (int)corner->walls.size();
            
            for (int j=0; j < sections; j++) {
                WallG2 wall = corner->walls[j];
                
                // For each wall, create one link on each side
                EdgeG2 edge1, edge2;
                edge1.v1 = edge2.v1 = &_G2Corners[wall.c1->i];
                edge1.v2 = edge2.v2 = &_G2Corners[wall.c2->i];
                edge1.section = j;
                edge2.section = (j+1)%sections;
                edge1.isAlongWall = 1;
                edge2.isAlongWall = 2;
                edge1.angle = edge2.angle = std::atan2(edge1.dy(), edge1.dx());
                edge1.dist = edge2.dist = std::sqrt(edge1.dx()*edge1.dx()+edge1.dy()*edge1.dy());
                
                // loss should be zero
                edge1.loss = edge2.loss = 0.0;
                
                corner->links.push_back(edge1);
                corner->links.push_back(edge2);
                
            }
        }
        
        // Create links to other corners and measurement points
        for (int j=0; j < _nG2totPoints; j++) {
            if (j==i) continue;
            
            PointG2 *p2 = _G2totPoints[j];
            EdgeG2 edge;
            edge.v1 = point;
            edge.v2 = p2;
            edge.angle = std::atan2(edge.dy(), edge.dx());
            edge.dist = std::sqrt(edge.dx()*edge.dx()+edge.dy()*edge.dy());
            edge.isAlongWall = 0;
            
            // First check if this is collinear to existing links
            bool addthis = true;
            int replaceid = -1;
            for (int k=0; k < point->links.size(); k++) {
                
                // Check if the link already exists
                //  - when this link goes along a wall
                if (point->links[k].v2 == p2) {
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
        }
        
    }

}

double DominantPath::getLoss(const EdgeG2 &edge) const {
    double x = edge.v1->x();
    double y = edge.v1->y();
    double dx = edge.dx();
    double dy = edge.dy();

    double loss = 0.0;
    
    for (int i=0; i < _flp->getNumWalls(); i++) {
        Wall *wall = _flp->getWallPtr(i);
        
        if (edge.v1->ref == wall->c1 || edge.v2->ref == wall->c1 || edge.v1->ref == wall->c2 || edge.v2->ref == wall->c2) continue;
        
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

int DominantPath::findSection(const CornerG2 *corner, const EdgeG2 &inedge) const {
    // First check if this comes along a wall
    if (inedge.isAlongWall) {
        // Find the corresponding wall
        int wallidx = -1;
        for (int i=0; i < corner->walls.size(); i++) {
            if (corner->walls[i].c2 == inedge.v1->ref) {
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
        if (inedge.isAlongWall==1) return (wallidx+1)%(corner->walls.size());
        else return wallidx;
    }
    
    // Else compute the angle
    double angle = atan2(-inedge.dy(),-inedge.dx());
    int sections = (int)corner->walls.size();
    int section = 0;
    for (int k=0; k < sections; k++) {
        if (angle < corner->walls[k].angle) break;
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
    
    if (point->isCorner()) {
        CornerG2 *corner = reinterpret_cast<CornerG2 *>(point);
        if (corner->walls.size()>1) std::cout << cornerLoss(corner, 1, 0) << std::endl;
    }
#endif
    
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", image );
    cv::waitKey(0);
}


