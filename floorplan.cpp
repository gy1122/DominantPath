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

