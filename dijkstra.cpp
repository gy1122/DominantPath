//
//  dijkstra.cpp
//  DominantPath
//
//  Created by Ger Yang on 2016/7/16.
//  Copyright (c) 2016年 Ger Yang. All rights reserved.
//

#include "floorplan.h"
#include <cmath>
#include <set>
#include <cassert>
#include <algorithm>
#include <cstdio>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define SHOW_DEBUG


// -------------------------------------
//  Class DominantPath (Dijkstra Initialization)
// -------------------------------------

// This function allocates memory for Dijkstra labels.
// Should only be called once when constructing G2.
// -------------------------------------
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

// This function resets Dijkstra labels.
// This is called in the beginning of Dijkstra function.
// -------------------------------------
void DominantPath::resetDijkstra() {
    for (int i=0; i < _nG2totPoints; i++) {
        PointG2 *point = _G2totPoints[i];
        if (point->isCorner()) {
            for (int j=0; j < point->links.size(); j++) {
                point->dlabels[j].from.p = 0;
                point->dlabels[j].from.i = 0;
                point->dlabels[j].val = INFINITY;
                point->dlabels[j].radar = INFINITY;
                point->dlabels[j].visited = false;
            }
        } else {
            point->dlabels[0].from.p = 0;
            point->dlabels[0].from.i = 0;
            point->dlabels[0].val = INFINITY;
            // radar has no meaning here
            point->dlabels[0].visited = false;
        }
    }
}

// -------------------------------------
//  Class DijkstraPoint
// -------------------------------------

DijkstraPoint::DijkstraPoint(PointG2 *p_, int i_):p(p_),i(i_) {
    // If p is a measurement point, then i should always be zero.
    if (p && !p->isCorner()) i = 0;
}

// -------------------------------------
//  Dijkstra-related helper functions
// -------------------------------------

DijkstraLabel &getDijkstraLabel(DijkstraPoint &p) {
    return p.p->dlabels[p.i];
}

double getDijkstraVal(const DijkstraPoint &p) {
    return p.p->dlabels[p.i].val;
}

double getDijkstraRadar(const DijkstraPoint &p) {
    return p.p->dlabels[p.i].radar;
}

// -------------------------------------
//  Class Priority_Queue
// -------------------------------------

bool Priority_Queue::mycmp_dijk_point::operator()(const DijkstraPoint &p1, const DijkstraPoint &p2) const {
    double v1 = getDijkstraVal(p1);
    double v2 = getDijkstraVal(p2);
    return (v1 < v2) || (v1 == v2 && p1.p < p2.p) || (v1 == v2 && p1.p == p2.p && p1.i < p2.i);
}

DijkstraPoint Priority_Queue::extract_min() {
    DijkstraPoint p = *(_queue.begin());
    _queue.erase(_queue.begin());
    return p;
}

void Priority_Queue::decrease_key(DijkstraPoint &p, double newval) {
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


// -------------------------------------
//  Class DominantPath (Dijkstra main)
// -------------------------------------

double anglediff(double ahead, double behind) {
    const double pi = std::abs(std::atan2(0,-1));
    if (ahead < behind) ahead += 2.0*pi;
    return (ahead - behind);
}

int incr(CornerG2 *corner, int i) {
    return corner->eincr(i);
}

int decr(CornerG2 *corner, int i) {
    return corner->edecr(i);
}

void DominantPath::Dijkstra_cornerSwipe(Priority_Queue &Q, DijkstraPoint dpoint, double start_idx, double angle, double lambda, bool cw) {
    
    // This function implements the "radar" data structure.
    
    CornerG2 *corner = reinterpret_cast<CornerG2 *>(dpoint.p);
    
    // Compute in sec
    int insec = corner->links[dpoint.i].section;
    
    // Direction
    int (*direc)(CornerG2 *, int) = cw? &incr : &decr;
    int (*rdirec)(CornerG2 *, int) = cw? &decr : &incr;
    
    // Terminating index
    int term_idx = rdirec(corner, start_idx);
    
    // Start a clockwise / counterclockwise swiping
    for (int i = start_idx; i != term_idx; i = direc(corner,i)) {
        
        DijkstraPoint epoint(corner, i);
        if (getDijkstraLabel(epoint).visited) break;
        
        EdgeG2 *edge = &corner->links[i];
        
        // Compute wall loss
        //  I should better use loss table
        int outsec = corner->links[i].section;
        double wall_loss = cornerLoss(corner, insec, outsec);
        
        double dAngle = cw? anglediff(edge->angle, angle) : anglediff(angle, edge->angle);
        double loss = wall_loss + _angleLoss * dAngle;
        
        double currad = getDijkstraRadar(epoint);
        double newrad = getDijkstraVal(dpoint) + loss;
        
        // Update radar but not update Dijkstra label for the socket
        if (newrad < currad) {
            //Q.decrease_key(epoint, newval);
            getDijkstraLabel(epoint).radar = newrad;
            getDijkstraLabel(epoint).from = dpoint;
            getDijkstraLabel(epoint).l_e = loss;
            getDijkstraLabel(epoint).d_e = 0.0;
            
            // Try to propagate Dijkstra label one step ahead
            DijkstraPoint epoint2(edge->target,
                                  edge->target_i);
            
            double newval2 = newrad + lambda * edge->dist + edge->loss;
            
            // check if this can be propagated one step forward
            if (newval2 < getDijkstraVal(epoint2)) {
                
                assert(!getDijkstraLabel(epoint2).visited);
                
                Q.decrease_key(epoint2, newval2);
                getDijkstraLabel(epoint2).from = epoint;
                getDijkstraLabel(epoint2).l_e = edge->loss;
                getDijkstraLabel(epoint2).d_e = edge->dist;
            }
        } else break;
    }
}

void DominantPath::relaxEdge(Priority_Queue &Q, DijkstraPoint dpoint, EdgeG2 edge, double lambda) {
    
    DijkstraPoint epoint(edge.target, edge.target_i);
    
    if (getDijkstraLabel(epoint).visited) return;
    
    // relax edge.target
    double val = getDijkstraVal(epoint);
    double newval = getDijkstraLabel(dpoint).val + (lambda * edge.dist + edge.loss);
    if (newval < val) {
        Q.decrease_key(epoint, newval);
        getDijkstraLabel(epoint).from = dpoint;
        getDijkstraLabel(epoint).d_e = edge.dist;
        getDijkstraLabel(epoint).l_e = edge.loss;
    }
}

void DominantPath::Dijkstra(double lambda, int s, int t) {
    
    const double pi = std::abs(std::atan2(0,-1));
    
    resetDijkstra();
    
    Priority_Queue Q;
    
    // We don't add all nodes into Q in the beginning but do that on the fly
    
    DijkstraPoint source(_G2totPoints[s]);
    getDijkstraLabel(source).val = 0.0;
    Q.add(source);
    
    while (Q.size() > 0) {
        
        DijkstraPoint dpoint = Q.extract_min();
        getDijkstraLabel(dpoint).visited = true;
        
        PointG2 *point = dpoint.p;
        
#ifdef SHOW_DEBUG
        printf("Extract (%3d, %3d)  Val: %f\n", point->i, dpoint.i, getDijkstraVal(dpoint));
#endif
        
        if (point->i == t) break;
        
        if (point->isCorner()) {
            
            double angle = point->links[dpoint.i].angle;
            if (angle > 0) angle -= pi;
            else angle += pi;
            
            int start_idx = point->searchIdx(angle);
            
            Dijkstra_cornerSwipe(Q, dpoint, point->eincr(start_idx), angle, lambda, true);
            Dijkstra_cornerSwipe(Q, dpoint, start_idx, angle, lambda, false);
        
        } else {
            
            for (int i=0; i < point->links.size(); i++) {
                EdgeG2 edge = point->links[i];
                relaxEdge(Q, dpoint, edge, lambda);
            }
        }
        
    }
}

void DominantPath::printDijkstra(int s, int t, double scale, double shift, double lambda) {
    
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
    double L = 0.0;
    double D = 0.0;
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
        printf("%3d %3d %f << %3d %3d :: %+.8f :: %f %f \n",
               dpoint.p->i,
               dpoint.i,
               dpoint.p == nextPoint.p? getDijkstraRadar(dpoint): getDijkstraVal(dpoint),
               
               nextPoint.p->i,
               nextPoint.i,
               //dpoint.p != nextPoint.p && nextPoint.p->isCorner()? getDijkstraRadar(nextPoint): getDijkstraVal(nextPoint),
               
               nextPoint.p->links[nextPoint.i].angle,
               getDijkstraLabel(dpoint).l_e,
               getDijkstraLabel(dpoint).d_e);
        
        L += getDijkstraLabel(dpoint).l_e;
        D += getDijkstraLabel(dpoint).d_e;
#endif
        
        dpoint = nextPoint;
    }
    
    printf("L=%f  D=%f      F=%f", L, D, L+lambda*D);
    
    
    
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", image );
    cv::waitKey(0);
}
