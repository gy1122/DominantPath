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
#include <cassert>


Floorplan::Floorplan(): _nCorners(0), _nWalls(0), _corners(0), _walls(0) {

}

Floorplan::~Floorplan() {
    if (_corners)   delete [] _corners;
    if (_walls)     delete [] _walls;
}

void Floorplan::genRandomFloorplan(int x, int y, double wallloss, double angleloss, double exterior_wallloss, unsigned seed) {

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
        _walls[curptr].loss = exterior_wallloss;
        curptr++;
    }

    for (int i=0; i < y; i++) {
        _walls[curptr].c1 = &_corners[i*(x+1)];
        _walls[curptr].c2 = &_corners[(i+1)*(x+1)];
        _walls[curptr].loss = exterior_wallloss;
        curptr++;
    }

    for (int i=0; i < y; i++) {
        for (int j=0; j < x; j++) {
            if (vedges[i*x+j]) {
                _walls[curptr].c1 = &_corners[i*(x+1)+j+1];
                _walls[curptr].c2 = &_corners[(i+1)*(x+1)+j+1];
                if (j == x-1) {
                    _walls[curptr].loss = exterior_wallloss;
                } else {
                    _walls[curptr].loss = wallloss;
                }
                curptr++;
            }
            if (hedges[i*x+j]) {
                _walls[curptr].c1 = &_corners[(i+1)*(x+1)+j];
                _walls[curptr].c2 = &_corners[(i+1)*(x+1)+j+1];
                _walls[curptr].loss = wallloss;
                if (i == y-1) {
                    _walls[curptr].loss = exterior_wallloss;
                } else {
                    _walls[curptr].loss = wallloss;
                }
                curptr++;
            }
        }
    }

    _angleLoss = angleloss;

    delete [] visited;
    delete [] visited_list;
    delete [] vedges;
    delete [] hedges;
    delete [] wvedges;
    delete [] whedges;
}

void Floorplan::genOffice1(int nx, int ny, double wx, double wy, double hall,
                           double wallloss, double angleloss,
                           double exterior_wallloss) {
    _nCorners = (2*nx+2)*(3*ny+1);
    _nWalls = (2*nx)*(3*ny+1) + (2*nx+2)*(2*ny) + 2*ny + 2;

    _corners = new Corner[_nCorners];
    _walls = new Wall[_nWalls];
    _angleLoss = angleloss;

    int p = 0;
    int w = 0;
    for (int j=0; j<3*ny+1; ++j) {
        for (int i=0; i<2*nx+2; ++i) {
            _corners[p].i = p;
            _corners[p].x = i * wx + ((i > nx) ? (hall - wx) : 0.0);
            _corners[p].y = j * wy + ((j+1)/3) * (hall - wy);
            if (i > 0 && !(i == nx+1 && j != 0 && j != 3*ny)) {
                _walls[w].c1 = &_corners[p-1];
                _walls[w].c2 = &_corners[p];
                if (j == 0 || j == 3*ny) {
                    _walls[w].loss = exterior_wallloss;
                } else {
                    _walls[w].loss = wallloss;
                }
                ++w;
            }
            if (j > 0 && !(j % 3 == 2 && i != 0 && i != 2*nx+1)) {
                _walls[w].c1 = &_corners[p-(2*nx+2)];
                _walls[w].c2 = &_corners[p];
                if (i == 0 || i == 2*ny+1) {
                    _walls[w].loss = exterior_wallloss;
                } else {
                    _walls[w].loss = wallloss;
                }
                ++w;
            }
            ++p;
        }
    }
    assert(w == _nWalls);
    assert(p == _nCorners);
}

void Floorplan::genOffice2(int nx, int ny, double wx, double wy, double hall,
                           double wallloss, double angleloss,
                           double exterior_wallloss) {
    _nCorners = 4 + (3*nx)*(ny+1);
    _nWalls = 4 + (2*nx)*(ny+1) + (3*nx)*ny;

    _corners = new Corner[_nCorners];
    _walls = new Wall[_nWalls];
    _angleLoss = angleloss;

    int p = 0;
    int w = 0;
    // 4 exterior corners and walls
    for (int j=0; j<2; j++) {
        for (int i=0; i<2; i++) {
            _corners[p].i = p;
            _corners[p].x = i * (2*nx*wx + (nx+1) * hall);
            _corners[p].y = j * (ny*wy + 2*hall);
            ++p;
        }
    }
    for (int i=0; i<2; i++) {
        _walls[w].c1 = &_corners[i];
        _walls[w].c2 = &_corners[i+2];
        _walls[w].loss = exterior_wallloss;
        ++w;
        _walls[w].c1 = &_corners[2*i];
        _walls[w].c2 = &_corners[2*i+1];
        _walls[w].loss = exterior_wallloss;
        ++w;
    }
    // interior walls
    for (int j=0; j<ny+1; ++j) {
        for (int i=0; i<3*nx; ++i) {
            _corners[p].i = p;
            _corners[p].x = hall + i * wx + (i/3) * (hall - wx);
            _corners[p].y = hall + j * wy;
            if (i % 3 != 0) {
                _walls[w].c1 = &_corners[p-1];
                _walls[w].c2 = &_corners[p];
                _walls[w].loss = wallloss;
                ++w;
            }
            if (j > 0) {
                _walls[w].c1 = &_corners[p-(3*nx)];
                _walls[w].c2 = &_corners[p];
                _walls[w].loss = wallloss;
                ++w;
            }
            ++p;
        }
    }
    assert(w == _nWalls);
    assert(p == _nCorners);
}

void Floorplan::genOffice3(int nx, int ny, double wx, double wy, double hall,
                           double wallloss, double angleloss,
                           double exterior_wallloss) {
    if (wx < wy) std::swap(wx,wy);

    int inx = (int)((nx*wy - 2*hall - 2*wx)/wy);
    double idelx = (nx*wy - hall - 2*wx - inx * wy)/2;
    int iny = (int)((ny*wy - 2*hall - 2*wx)/wy);
    double idely = (ny*wy - hall - 2*wx - iny * wy)/2;

    double totx = nx*wy + 2*wx + hall;
    double toty = ny*wy + 2*wx + hall;

    _nCorners = 2*(2*nx+4) + 2*(2*ny+4) + 2*(2*inx+4) + 2*(2*iny+4) - 2;
    _nWalls = 2*(3*nx+5) + 2*(3*ny+5) + 2*(3*inx+5) + 2*(3*iny+5) - 4;

    _corners = new Corner[_nCorners];
    _walls = new Wall[_nWalls];
    _angleLoss = angleloss;

    int p = 0;
    int w = 0;
    // clumsy, but just do each side at a time.
    // the outer top
    _walls[w] = {&_corners[p+0], &_corners[p+2], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], wallloss}; ++w;
    _corners[p] = {p, 0.0, wx + hall/2}; ++p;
    _corners[p] = {p, wx, wx + hall/2}; ++p;
    _corners[p] = {p, 0.0, 0.0}; ++p;
    _corners[p] = {p, wx, wx}; ++p;
    for (int i=0; i<nx; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], exterior_wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
        _corners[p] = {p, wx + hall/2 + i*wy, 0.0}; ++p;
        _corners[p] = {p, wx + hall/2 + i*wy, wx}; ++p;
    }

    // the outer right side
    _walls[w] = {&_corners[p+0], &_corners[p+2], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], wallloss}; ++w;
    _corners[p] = {p, totx - wx - hall/2, 0.0}; ++p;
    _corners[p] = {p, totx - wx - hall/2, wx}; ++p;
    _corners[p] = {p, totx, 0.0}; ++p;
    _corners[p] = {p, totx - wx, wx}; ++p;
    for (int i=0; i<ny; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], exterior_wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
        _corners[p] = {p, totx, wx + hall/2 + i*wy}; ++p;
        _corners[p] = {p, totx - wx, wx + hall/2 + i*wy}; ++p;
    }

    // the outer bottom
    _walls[w] = {&_corners[p+0], &_corners[p+2], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], wallloss}; ++w;
    _corners[p] = {p, totx, toty - wx - hall/2}; ++p;
    _corners[p] = {p, totx - wx, toty - wx - hall/2}; ++p;
    _corners[p] = {p, totx, toty}; ++p;
    _corners[p] = {p, totx - wx, toty - wx}; ++p;
    for (int i=0; i<nx; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], exterior_wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
        _corners[p] = {p, totx - wx - hall/2 - i*wy, toty}; ++p;
        _corners[p] = {p, totx - wx - hall/2 - i*wy, toty - wx}; ++p;
    }

    // the outer left side
    _walls[w] = {&_corners[p+0], &_corners[p+2], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], wallloss}; ++w;
    _corners[p] = {p, wx + hall/2, toty}; ++p;
    _corners[p] = {p, wx + hall/2, toty - wx}; ++p;
    _corners[p] = {p, 0.0, toty}; ++p;
    _corners[p] = {p, wx, toty - wx}; ++p;
    for (int i=0; i<ny; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], exterior_wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
        _corners[p] = {p, 0.0, toty - wx - hall/2 - i*wy}; ++p;
        _corners[p] = {p, wx, toty - wx - hall/2 - i*wy}; ++p;
    }
    // fix final wall connections!
    _walls[w-2].c2 = &_corners[0];
    _walls[w-1].c2 = &_corners[1];

    int inner_p0 = p;
    double ibase = wx + hall;
    double itotx = ibase + inx*wy + 2*wx + 2*idelx;
    double itoty = ibase + iny*wy + 2*wx + 2*idely;
    // the inner top
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _corners[p] = {p, ibase, ibase + wx + idely}; ++p;
    _corners[p] = {p, ibase + wx, ibase + wx + idely}; ++p;
    for (int i=0; i<inx; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
        _corners[p] = {p, ibase + wx + idelx + i*wy, ibase}; ++p;
        _corners[p] = {p, ibase + wx + idelx + i*wy, ibase + wx}; ++p;
    }

    // the inner right side
    _walls[w] = {&_corners[p+0], &_corners[p+2], wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], exterior_wallloss}; ++w;
    _corners[p] = {p, itotx - wx - idelx, ibase}; ++p;
    _corners[p] = {p, itotx - wx - idelx, ibase + wx}; ++p;
    _corners[p] = {p, itotx, ibase}; ++p;
    _corners[p] = {p, itotx - wx, ibase + wx}; ++p;
    for (int i=0; i<iny; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
        _corners[p] = {p, itotx, ibase + wx + idely + i*wy}; ++p;
        _corners[p] = {p, itotx - wx, ibase + wx + idely + i*wy}; ++p;
    }

    // the inner bottom
    _walls[w] = {&_corners[p+0], &_corners[p+2], wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], exterior_wallloss}; ++w;
    _corners[p] = {p, itotx, itoty - wx - idely}; ++p;
    _corners[p] = {p, itotx - wx, itoty - wx - idely}; ++p;
    _corners[p] = {p, itotx, itoty}; ++p;
    _corners[p] = {p, itotx - wx, itoty - wx}; ++p;
    for (int i=0; i<inx; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
        _corners[p] = {p, itotx - wx - idelx - i*wy, itoty}; ++p;
        _corners[p] = {p, itotx - wx - idelx - i*wy, itoty - wx}; ++p;
    }

    // the inner left side
    _walls[w] = {&_corners[p+0], &_corners[p+2], wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], exterior_wallloss}; ++w;
    _corners[p] = {p, ibase + wx + idelx, itoty}; ++p;
    _corners[p] = {p, ibase + wx + idelx, itoty - wx}; ++p;
    _corners[p] = {p, ibase, itoty}; ++p;
    _corners[p] = {p, ibase + wx, itoty - wx}; ++p;
    for (int i=0; i<iny; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
        _corners[p] = {p, ibase, itoty - wx - idely - i*wy}; ++p;
        _corners[p] = {p, ibase + wx, itoty - wx - idely - i*wy}; ++p;
    }
    // fix final wall connections!
    _walls[w-2].c2 = &_corners[inner_p0];
    _walls[w-1].c2 = &_corners[inner_p0+1];

    assert(w == _nWalls);
    assert(p == _nCorners);
}

void Floorplan::genOffice4(int nx, int ny, double wx, double wy, double hall,
                           double wallloss, double angleloss,
                           double exterior_wallloss) {
    if (wx < wy) std::swap(wx,wy);

    int inx = (int)((nx*wy - 2*hall - 2*wx)/wy);
    double idelx = (nx*wy - hall - 2*wx - inx * wy)/2;
    int iny = (int)((ny*wy - 2*hall - 2*wx)/wy);
    double idely = (ny*wy - hall - 2*wx - iny * wy)/2;

    double totx = nx*wy + 2*wx + hall;
    double toty = ny*wy + 2*wx + hall;

    _nCorners = 2*(2*nx+4) + 2*(2*ny+4) + 2*(2*inx+4) + 2*(2*iny+4) - 4;
    _nWalls = 2*(3*nx+5) + 2*(3*ny+5) + 2*(3*inx+5) + 2*(3*iny+5) - 8;

    _corners = new Corner[_nCorners];
    _walls = new Wall[_nWalls];
    _angleLoss = angleloss;

    int p = 0;
    int w = 0;
    // clumsy, but just do each side at a time.
    // the outer top
    _walls[w] = {&_corners[p+0], &_corners[p+2], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], wallloss}; ++w;
    _corners[p] = {p, 0.0, wx + hall/2}; ++p;
    _corners[p] = {p, wx, wx + hall/2}; ++p;
    _corners[p] = {p, 0.0, 0.0}; ++p;
    _corners[p] = {p, wx, wx}; ++p;
    for (int i=0; i<nx; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], exterior_wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
        _corners[p] = {p, wx + hall/2 + i*wy, 0.0}; ++p;
        _corners[p] = {p, wx + hall/2 + i*wy, wx}; ++p;
    }

    // the outer right side
    _walls[w] = {&_corners[p+0], &_corners[p+2], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], wallloss}; ++w;
    _corners[p] = {p, totx - wx - hall/2, 0.0}; ++p;
    _corners[p] = {p, totx - wx - hall/2, wx}; ++p;
    _corners[p] = {p, totx, 0.0}; ++p;
    _corners[p] = {p, totx - wx, wx}; ++p;
    for (int i=0; i<ny; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], exterior_wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
        _corners[p] = {p, totx, wx + hall/2 + i*wy}; ++p;
        _corners[p] = {p, totx - wx, wx + hall/2 + i*wy}; ++p;
    }

    // the outer bottom
    _walls[w] = {&_corners[p+0], &_corners[p+2], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], wallloss}; ++w;
    _corners[p] = {p, totx, toty - wx - hall/2}; ++p;
    _corners[p] = {p, totx - wx, toty - wx - hall/2}; ++p;
    _corners[p] = {p, totx, toty}; ++p;
    _corners[p] = {p, totx - wx, toty - wx}; ++p;
    for (int i=0; i<nx; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], exterior_wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
        _corners[p] = {p, totx - wx - hall/2 - i*wy, toty}; ++p;
        _corners[p] = {p, totx - wx - hall/2 - i*wy, toty - wx}; ++p;
    }

    // the outer left side
    _walls[w] = {&_corners[p+0], &_corners[p+2], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], wallloss}; ++w;
    _corners[p] = {p, wx + hall/2, toty}; ++p;
    _corners[p] = {p, wx + hall/2, toty - wx}; ++p;
    _corners[p] = {p, 0.0, toty}; ++p;
    _corners[p] = {p, wx, toty - wx}; ++p;
    for (int i=0; i<ny; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], exterior_wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], wallloss}; ++w;
        _corners[p] = {p, 0.0, toty - wx - hall/2 - i*wy}; ++p;
        _corners[p] = {p, wx, toty - wx - hall/2 - i*wy}; ++p;
    }
    // fix final wall connections!
    _walls[w-2].c2 = &_corners[0];
    _walls[w-1].c2 = &_corners[1];

    int inner_p0 = p;
    double ibase = wx + hall;
    double itotx = ibase + inx*wy + 2*wx + 2*idelx;
    double itoty = ibase + iny*wy + 2*wx + 2*idely;
    // the inner top
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _corners[p] = {p, ibase, ibase + wx + idely}; ++p;
    _corners[p] = {p, ibase + wx, ibase + wx + idely}; ++p;
    for (int i=0; i<inx; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
        _corners[p] = {p, ibase + wx + idelx + i*wy, ibase}; ++p;
        _corners[p] = {p, ibase + wx + idelx + i*wy, ibase + wx}; ++p;
    }

    // the inner right side
    _walls[w] = {&_corners[p+0], &_corners[p+2], wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], exterior_wallloss}; ++w;
    _corners[p] = {p, itotx - wx - idelx, ibase}; ++p;
    _corners[p] = {p, itotx - wx - idelx, ibase + wx}; ++p;
    _corners[p] = {p, itotx, ibase}; ++p;
    _corners[p] = {p, itotx - wx, ibase + wx}; ++p;
    for (int i=0; i<iny; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
        _corners[p] = {p, itotx, ibase + wx + idely + i*wy}; ++p;
        _corners[p] = {p, itotx - wx, ibase + wx + idely + i*wy}; ++p;
    }

    // the inner bottom
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _corners[p] = {p, itotx, itoty - wx - idely}; ++p;
    _corners[p] = {p, itotx - wx, itoty - wx - idely}; ++p;
    for (int i=0; i<inx; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
        _corners[p] = {p, itotx - wx - idelx - i*wy, itoty}; ++p;
        _corners[p] = {p, itotx - wx - idelx - i*wy, itoty - wx}; ++p;
    }

    // the inner left side
    _walls[w] = {&_corners[p+0], &_corners[p+2], wallloss}; ++w;
    _walls[w] = {&_corners[p+2], &_corners[p+4], wallloss}; ++w;
    _walls[w] = {&_corners[p+0], &_corners[p+1], wallloss}; ++w;
    _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
    _walls[w] = {&_corners[p+3], &_corners[p+5], exterior_wallloss}; ++w;
    _corners[p] = {p, ibase + wx + idelx, itoty}; ++p;
    _corners[p] = {p, ibase + wx + idelx, itoty - wx}; ++p;
    _corners[p] = {p, ibase, itoty}; ++p;
    _corners[p] = {p, ibase + wx, itoty - wx}; ++p;
    for (int i=0; i<iny; ++i) {
        _walls[w] = {&_corners[p], &_corners[p+1], wallloss}; ++w;
        _walls[w] = {&_corners[p], &_corners[p+2], wallloss}; ++w;
        _walls[w] = {&_corners[p+1], &_corners[p+3], exterior_wallloss}; ++w;
        _corners[p] = {p, ibase, itoty - wx - idely - i*wy}; ++p;
        _corners[p] = {p, ibase + wx, itoty - wx - idely - i*wy}; ++p;
    }
    // fix final wall connections!
    _walls[w-2].c2 = &_corners[inner_p0];
    _walls[w-1].c2 = &_corners[inner_p0+1];

    assert(w == _nWalls);
    assert(p == _nCorners);
}

int Floorplan::save(const char *filename) const {

    FILE *file;
    file = fopen(filename, "w+");

    if (file == NULL) return -1;

    fprintf(file, "%d %d %f\n", _nCorners, _nWalls, _angleLoss);
    for (int i = 0; i < _nCorners; i++) {
        fprintf(file, "c %d %lf %lf\n", _corners[i].i, _corners[i].x, _corners[i].y);
    }
    for (int i = 0; i < _nWalls; i++) {
        fprintf(file, "w %d %d %lf\n", _walls[i].c1->i, _walls[i].c2->i, _walls[i].loss);
    }
    fclose(file);

    return 0;
}

int Floorplan::load(const char *filename) {
    FILE *file;
    file = fopen(filename, "r");

    if (file == NULL) return -1;

    if (fscanf(file, "%d %d %lf", &_nCorners, &_nWalls, &_angleLoss) != 3) return -1;

    _corners = new Corner[_nCorners];
    _walls = new Wall[_nWalls];

    int wptr = 0;
    while ( ! feof(file) ) {
        char buf;
        if (fscanf(file, "%c", &buf) != 1) return -1;
        if (buf == 'c') {
            int ci;
            double cx, cy;
            if (fscanf(file, "%d %lf %lf", &ci, &cx, &cy) != 3) return -1;
            _corners[ci].i = ci;
            _corners[ci].x = cx;
            _corners[ci].y = cy;
        }
        if (buf == 'w') {
            int c1, c2;
            double l;
            if (fscanf(file, "%d %d %lf", &c1, &c2, &l) != 3) return -1;
            _walls[wptr].c1 = &_corners[c1];
            _walls[wptr].c2 = &_corners[c2];
            _walls[wptr].loss = l;
            wptr++;
        }
    }

    assert(wptr == _nWalls);

    return 0;
}

double Floorplan::getWidth() const {
    double width = 0;
    for (int i=0; i<_nCorners; ++i) {
        if (_corners[i].x > width) width = _corners[i].x;
    }
    return width;
}

double Floorplan::getHeight() const {
    double height = 0;
    for (int i=0; i<_nCorners; ++i) {
        if (_corners[i].y > height) height = _corners[i].y;
    }
    return height;
}
