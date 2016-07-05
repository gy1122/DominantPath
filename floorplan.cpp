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

#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PRINT_MAP

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
    double *wvedges = new double[y*(x-1)];
    double *whedges = new double[x*(y-1)];
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
        int min_dir = -1;
        for (int i=0; i < n_visited; i++) {
            int j = visited_list[i];
            int cx = j % x;
            int cy = j / x;
            
            // check adjacent nodes
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
    
#ifdef PRINT_MAP
    
    std::cout << '+';
    for (int xx=0; xx<x; xx++)
        std::cout << "-+";
    std::cout << std::endl;
    
    for (int yy=0; yy<y-1; yy++) {
        std::cout << '|';
        for (int xx=0; xx<x-1; xx++) {
            if (vedges[yy*x+xx]) std::cout << " |";
            else std::cout << "  ";
        }
        std::cout << " |" << std::endl;
        std::cout << '+';
        for (int xx=0; xx<x; xx++) {
            if (hedges[yy*x+xx]) std::cout << "-+";
            else std::cout << " +";
        }
        std::cout << std::endl;
    }
    
    std::cout << '|';
    for (int xx=0; xx<x-1; xx++) {
        if (vedges[(y-1)*x+xx]) std::cout << " |";
        else std::cout << "  ";
    }
    std::cout << " |" << std::endl;
    
    std::cout << '+';
    for (int xx=0; xx<x; xx++)
        std::cout << "-+";
    std::cout << std::endl;
    
#endif
    
    // OK, now we have a random spanning tree.
    // Next, generate the floorplan graph:
    
    // -- Generate corners
    _nCorners = (x+1)*(y+1);
    _corners = new Corner[_nCorners];
    for (int i=0; i<(x+1)*(y+1); i++) {
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

int Floorplan::getNumCorners() const {
    return _nCorners;
}

Corner *Floorplan::getCornerPtr(int i) const {
    return &_corners[i];
}

int Floorplan::getNumWalls() const {
    return _nWalls;
}

Wall *Floorplan::getWallPtr(int i) const {
    return &_walls[i];
}


DominantPath::DominantPath(Floorplan *flp, int nmpt, Point *mpts): _flp(flp), _nmPoints(nmpt), _mPoints(mpts), _nG1Points(0), _G1Points(0) {
    
}

DominantPath::~DominantPath() {
    if (_G1Points) delete [] _G1Points;
}

void DominantPath::generateG1() {
    
    
    _nG1Points = _nmPoints + _flp->getNumCorners();
    _G1Points = new PointG1[_nG1Points];
    
    // First create nodes for G1
    int idx = 0;
    for (int i=0; i < _nmPoints; i++) {
        _G1Points[idx].ref = &_mPoints[i];
        _G1Points[idx].isCorner = false;
        idx++;
    }
    for (int i=0; i < _flp->getNumCorners(); i++) {
        _G1Points[idx].ref = _flp->getCornerPtr(i);
        _G1Points[idx].isCorner = true;
        idx++;
    }
    
    // Then, create edges
    for (int i=0; i < _nG1Points; i++) {
        for (int j=0; j < _nG1Points; j++) {
            if (i==j) continue;
            
            // Create edge i -> j
            EdgeG1 edge;
            edge.dest = &_G1Points[j];
            
            // Calculate distance and angle
            double dx = _G1Points[j].ref->x - _G1Points[i].ref->x;
            double dy = _G1Points[j].ref->y - _G1Points[i].ref->y;
            edge.dist = std::sqrt(dx*dx+dy*dy);
            edge.angle = std::atan2(dy, dx);
            
            // Calculate loss
            
            _G1Points[i].links.push_back(edge);
        }
    }
}

// Check if the data structure is correct
void DominantPath::printG1() {
    cv::Mat image(800,800,CV_8UC1,cv::Scalar(255));
    int scale = 150;
    int shift = 5;
    
    for (int i=0; i < _nG1Points; i++) {
        PointG1 *point = &_G1Points[i];
        cv::Point center(point->ref->x*scale+shift, point->ref->y*scale+shift);
        cv::circle(image, center, 3, cv::Scalar(0));
        
        for (int j=0; j < point->links.size(); j++) {
            EdgeG1 *edge = &point->links[j];
            if (edge->dist > 3) continue;
            double x = point->ref->x + edge->dist * std::cos(edge->angle);
            double y = point->ref->y + edge->dist * std::sin(edge->angle);
            cv::Point p2(x*scale+shift, y*scale+shift);
            cv::line(image, center, p2, cv::Scalar(100));
        }
    }
    
    for (int i=0; i < _flp->getNumWalls(); i++) {
        Wall *wall = _flp->getWallPtr(i);
        cv::Point p1(wall->c1->x*scale+shift, wall->c1->y*scale+shift);
        cv::Point p2(wall->c2->x*scale+shift, wall->c2->y*scale+shift);
        cv::line(image, p1, p2, cv::Scalar(0), 2);
    }
    
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", image );
    cv::waitKey(0);
}


