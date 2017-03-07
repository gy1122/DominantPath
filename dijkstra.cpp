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
#include <queue>
#include <cassert>
#include <algorithm>
#include <cstdio>
#include <iostream>

//#define SHOW_DEBUG
#define SHOW_DEBUG2


// -------------------------------------
//  Class DominantPath (Dijkstra Initialization)
// -------------------------------------

// This function allocates memory for Dijkstra labels.
// Should only be called once when constructing G2.
// -------------------------------------
void DominantPath::initDijkstra() {
    for (int i=0; i < _nG2totPoints; i++) {
        PointG2 *point = _G2totPoints[i];
        // Avoid memory leak if initDijkstra is called twice
        if (point->dlabels) {
          delete [] point->dlabels;
        }
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
            for (int j=0; j < (int) point->links.size(); j++) {
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

double DominantPath::parametricWeight(double l, double d, double lambda) {
    if (lambda == INFINITY) return d;
    else return l + lambda * d;
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

int wincr(CornerG2 *corner, int i) {
    return corner->wincr(i);
}

int wdecr(CornerG2 *corner, int i) {
    return corner->wdecr(i);
}

int DominantPath::Dijkstra_cornerSwipe(Priority_Queue &Q, DijkstraPoint dpoint, int start_idx, double angle, double lambda, bool cw) {

    // This function implements the "radar" data structure.

    int count = 0;

    CornerG2 *corner = reinterpret_cast<CornerG2 *>(dpoint.p);

    // Compute in sec
    int insec = corner->links[dpoint.i].section;

    // Wall Direction
    int (*wdirec)(CornerG2 *, int) = cw? &wincr : &wdecr;
    // Direction
    int (*direc)(CornerG2 *, int) = cw? &incr : &decr;

    int section = corner->links[start_idx].section;
    while (start_idx != dpoint.i) {
        // Compute wall loss
        //  I should better use loss table
        double wall_loss = cornerLoss(corner, insec, section);
        int term_idx = cw ? corner->section_start[wdirec(corner, section)]
            : direc(corner,corner->section_start[section]);
        // do we reach dpoint.i before term_idx?
        if (corner->between(cw, start_idx, dpoint.i, term_idx)) {
          term_idx = dpoint.i;
        }
        count += Dijkstra_sectionSwipe(Q, dpoint, start_idx, term_idx,
                                       wall_loss, angle, lambda, cw);
        start_idx = term_idx;
        section = wdirec(corner, section);
    }

    return count;
}

// This function implements the corner swipe within one section between walls
// processes [start_idx,term_idx) (circularly); if start_idx == term_idx,
// this means the full set, not the empty set.
int DominantPath::Dijkstra_sectionSwipe(Priority_Queue &Q, DijkstraPoint dpoint, int start_idx, int term_idx, double wall_loss, double angle, double lambda, bool cw) {
    CornerG2 *corner = reinterpret_cast<CornerG2 *>(dpoint.p);

    // Direction
    int (*direc)(CornerG2 *, int) = cw? &incr : &decr;

    int count = 0;

    // Start a clockwise / counterclockwise swiping
    for (int i = start_idx; i != term_idx; i = direc(corner,i)) {

        DijkstraPoint epoint(corner, i);
        if (getDijkstraLabel(epoint).visited) break;

        EdgeG2 *edge = &corner->links[i];

        double dAngle = cw? anglediff(edge->angle, angle) : anglediff(angle, edge->angle);
        double loss = wall_loss + _flp->getAngleLoss() * dAngle;

        double currad = getDijkstraRadar(epoint);
        double newrad = getDijkstraVal(dpoint) + parametricWeight(loss, 0, lambda);

        count++;

        // Update radar but not update Dijkstra label for the socket
        if (newrad < currad) {
            //Q.decrease_key(epoint, newval);
            getDijkstraLabel(epoint).radar = newrad;
            //getDijkstraLabel(epoint).from = dpoint;
            //getDijkstraLabel(epoint).l_e = loss;
            //getDijkstraLabel(epoint).d_e = 0.0;

            // Try to propagate Dijkstra label one step ahead
            DijkstraPoint epoint2(edge->target,
                                  edge->target_i);

            double newval2 = newrad + parametricWeight(edge->loss, edge->dist, lambda);
            double newdist = getDijkstraLabel(dpoint).dist + edge->dist;

            // check if this can be propagated one step forward
            if (newdist <= _Dijkstra_dist_constraint && newval2 < getDijkstraVal(epoint2)) {

                assert(!getDijkstraLabel(epoint2).visited);

                Q.decrease_key(epoint2, newval2);
                getDijkstraLabel(epoint2).dist = newdist;
                getDijkstraLabel(epoint2).from = dpoint;
                getDijkstraLabel(epoint2).l_e = loss + edge->loss;
                getDijkstraLabel(epoint2).d_e = edge->dist;
            }
        } else break;
    }

    return count;
}

void DominantPath::relaxEdge(Priority_Queue &Q, DijkstraPoint dpoint, EdgeG2 edge, double lambda) {

    DijkstraPoint epoint(edge.target, edge.target_i);

    if (getDijkstraLabel(epoint).visited) return;

    // relax edge.target
    double val = getDijkstraVal(epoint);
    double newval = getDijkstraLabel(dpoint).val + parametricWeight(edge.loss, edge.dist, lambda);
    double newdist = getDijkstraLabel(dpoint).dist + edge.dist;

    if (newdist <= _Dijkstra_dist_constraint && newval < val) {
        Q.decrease_key(epoint, newval);
        getDijkstraLabel(epoint).dist = newdist;
        getDijkstraLabel(epoint).from = dpoint;
        getDijkstraLabel(epoint).d_e = edge.dist;
        getDijkstraLabel(epoint).l_e = edge.loss;
    }
}

int DominantPath::Dijkstra(double lambda, int s, int t, Path &path) {

    const double pi = std::abs(std::atan2(0,-1));

    resetDijkstra();

    Priority_Queue Q;

    // We don't add all nodes into Q in the beginning but do that on the fly

    DijkstraPoint source(_G2totPoints[s]);
    getDijkstraLabel(source).val = 0.0;
    getDijkstraLabel(source).dist = 0.0;
    Q.add(source);

    // This is the counter for the number of relaxations
    int count = 0;

    while (Q.size() > 0) {

        DijkstraPoint dpoint = Q.extract_min();
        getDijkstraLabel(dpoint).visited = true;

        PointG2 *point = dpoint.p;

        if (point->i == t) break;

        if (point->isCorner()) {

            double angle = point->links[dpoint.i].angle;
            if (angle > 0) angle -= pi;
            else angle += pi;

            int start_idx = point->searchIdx(angle);

            count += Dijkstra_cornerSwipe(Q, dpoint, point->eincr(start_idx), angle, lambda, true);
            count += Dijkstra_cornerSwipe(Q, dpoint, start_idx, angle, lambda, false);

        } else if (point->i == s){

            for (int i=0; i < (int) point->links.size(); i++) {
                EdgeG2 edge = point->links[i];
                relaxEdge(Q, dpoint, edge, lambda);
                count++;
            }

        }

    }

    backTrack(lambda, s, t, path);

    // Return the number of relaxations
    return count;

}

int DominantPath::Dijkstra_all_dest(double lambda, Path* &paths) {
    
    const double pi = std::abs(std::atan2(0,-1));
    
    resetDijkstra();
    
    Priority_Queue Q;
    int found_dest = 0;
    
    // We don't add all nodes into Q in the beginning but do that on the fly
    // source is 0
    DijkstraPoint source(_G2totPoints[0]);
    getDijkstraLabel(source).val = 0.0;
    getDijkstraLabel(source).dist = 0.0;
    Q.add(source);
    
    // This is the counter for the number of relaxations
    int count = 0;
    
    while (Q.size() > 0) {
        
        // Extract one point from the priority queue
        DijkstraPoint dpoint = Q.extract_min();
        getDijkstraLabel(dpoint).visited = true;
        
        PointG2 *point = dpoint.p;
        
        // Check if we have all Dijkstra labels
        if (!point->isCorner()) {
            found_dest++;
            if (found_dest == _nG2Points) break;
        }
        
        if (point->isCorner()) {
            
            double angle = point->links[dpoint.i].angle;
            if (angle > 0) angle -= pi;
            else angle += pi;
            
            int start_idx = point->searchIdx(angle);
            
            count += Dijkstra_cornerSwipe(Q, dpoint, point->eincr(start_idx), angle, lambda, true);
            count += Dijkstra_cornerSwipe(Q, dpoint, start_idx, angle, lambda, false);
            
        } else if (point->i == 0) {
            
            for (int i=0; i < (int) point->links.size(); i++) {
                EdgeG2 edge = point->links[i];
                relaxEdge(Q, dpoint, edge, lambda);
                count++;
            }
            
        }
        
    }
    
    // The following assertion might fail if we have distance constraint.
    //assert(found_dest == _nG2Points);
    
    for (int i = 0; i < _nG2Points-1; i++) {
        backTrack(lambda, 0, i+1, paths[i]);
    }
    
    // Return the number of relaxations
    return count;
}

void DominantPath::backTrack(double lambda, int s, int t, Path &path) {

    path.reset();
    DijkstraPoint ptr(_G2totPoints[t], 0);
    
    if (getDijkstraLabel(ptr).from.p == 0) { // not reachable
        path.L = INFINITY;
        path.D = INFINITY;
        return;
    }

    while (ptr.p->i != s) {
        DijkstraPoint prevPoint = getDijkstraLabel(ptr).from;

        path.v.push_back(ptr);

#ifdef SHOW_DEBUG
        printf("%3d %3d %f << %3d %3d :: %+.8f :: %f %f \n",
               ptr.p->i,
               ptr.i,
               //ptr.p == prevPoint.p? getDijkstraRadar(ptr): getDijkstraVal(ptr),
               getDijkstraVal(ptr),

               prevPoint.p->i,
               prevPoint.i,
               //dpoint.p != nextPoint.p && nextPoint.p->isCorner()? getDijkstraRadar(nextPoint): getDijkstraVal(nextPoint),

               prevPoint.p->links[prevPoint.i].angle,
               getDijkstraLabel(ptr).l_e,
               getDijkstraLabel(ptr).d_e);
#endif

        path.L += getDijkstraLabel(ptr).l_e;
        path.D += getDijkstraLabel(ptr).d_e;

        ptr = prevPoint;
    }

    path.v.push_back(ptr);

#ifdef SHOW_DEBUG
    printf("L=%f  D=%f      F=%f\n", path.L, path.D, path.L+lambda*path.D);
#endif

    std::reverse(path.v.begin(), path.v.end());
}

bool mycmp_bp(const Path &p1, const Path &p2) {
    return p1.L < p2.L;
}

// This function finds all breakpoints for s-t paths.
int DominantPath::BreakPoints(int s, int t, int limit, Path *paths, int &npaths) {
    int count = 0;

    typedef std::pair<int,int> spair;
    std::queue<spair> queue;

    npaths = 0;

    // L + lambda * D

    count += Dijkstra(0, s, t, paths[npaths++]); // L
    count += Dijkstra(INFINITY, s, t, paths[npaths++]);  // D
    
    // If only one path found, then return the only path
    if (paths[0].D == paths[1].D && paths[0].L == paths[1].L) {
        npaths--;
        return count;
    }
    
    queue.push(spair(0,1));

    while (!queue.empty()) {

        if (npaths == limit) {
#ifdef SHOW_DEBUG
            printf("Too many breakpoints.\n");
#endif
            break;
        }

        spair sp = queue.front();
        queue.pop();

        double dL = (paths[sp.first].L - paths[sp.second].L);
        double dD = (paths[sp.first].D - paths[sp.second].D);
        if (std::abs(dL) < TINY || std::abs(dD) < TINY) {
            // avoid new lambda=0 or lambda=inf queries
            continue;
        }
        double lambda = -dL/dD;

        assert(dL <= 0);
        assert(dD >= 0);

        Path path;
        count += Dijkstra(lambda, s, t, path);

#ifdef SHOW_DEBUG
        printf("(%d, %d) %f: (%f, %f) -- (%f, %f)  ",
               sp.first, sp.second, lambda,
               paths[sp.first].L, paths[sp.first].D,
               paths[sp.second].L, paths[sp.second].D);
        printf("path %d: L=%f   D=%f \n", npaths, path.L, path.D);
#endif

        if (std::abs(paths[sp.first].L-path.L) < TINY &&
            std::abs(paths[sp.first].D-path.D) < TINY)
            continue;
        if (std::abs(paths[sp.second].L-path.L) < TINY &&
            std::abs(paths[sp.second].D-path.D) < TINY)
            continue;

#ifdef SHOW_DEBUG
        printf("add path\n");
#endif

        paths[npaths++] = path;
        queue.push(spair(sp.first, npaths-1));
        queue.push(spair(npaths-1, sp.second));
    }

    std::sort(paths, paths+npaths, mycmp_bp);

    return count;
}

int DominantPath::Approx_all_dest(double p, double step, Path *&paths) {
    int count = 0;
    
    Path *tmpPaths = new Path[_nG2Points-1];
    double *vals = new double[_nG2Points-1];
    double lambda = 0.0;
    double D_min = INFINITY;
    double D_max = 0.0;
    //double bar_f = 0.0;
    double min_lambda = 0.0;
    
    int numDijkstra = 0;
    
    for (int i = 0; i < _nG2Points - 1; i++) {
        vals[i] = INFINITY;
        paths[i].reset();
    }
    
    // f = L + p ln D/D_min + p ln D_min

    // Algorithm: apply Dijkstra on (  L + p e^{-i \eps/p} D/D_min )
    
    // Find D_min (the min dist)
    count += Dijkstra_all_dest(INFINITY, tmpPaths);
    for (int i = 0; i < _nG2Points - 1; i++) {
        if (tmpPaths[i].D < D_min) D_min = tmpPaths[i].D;
        
        double obj = tmpPaths[i].L + p * std::log(tmpPaths[i].D);
        vals[i] = obj;
        paths[i] = tmpPaths[i];
    }
    
    numDijkstra++;
    
    // Find D_max
    count += Dijkstra_all_dest(0, tmpPaths);
    for (int i = 0; i < _nG2Points - 1; i++) {
        if (tmpPaths[i].D > D_max) D_max = tmpPaths[i].D;
        
        double obj = tmpPaths[i].L + p * std::log(tmpPaths[i].D);
        if (obj < vals[i]) {
            vals[i] = obj;
            paths[i] = tmpPaths[i];
        }
    }
    
    
    // bar_f: the upper bound on the objective given by arbitrary path.  Choose the largest one among all measurement points
    /*
    for (int i = 0; i < _nG2Points - 1; i++) {
        double tmpVal = tmpPaths[i].L + p * std::log(tmpPaths[i].D / D_min);
        if ( tmpVal > bar_f ) bar_f = tmpVal;
    }
     */
    
    //min_lambda = p * exp(-bar_f/p) / D_min;
    min_lambda = p / D_max;
    
    printf("D_max: %f\n", D_max);
    //assert(D_min/exp(-bar_f/p) <= D_max);
    
    
    //
    lambda = p / D_min / step;
    
    while (lambda >= min_lambda) {
        
        lambda = lambda * step;
        
#ifdef SHOW_DEBUG2
        printf("lambda=%f\n", lambda);
#endif
        
        _Dijkstra_dist_constraint = p / lambda;
        
        count += Dijkstra_all_dest(lambda, tmpPaths);
        for (int i = 0; i < _nG2Points - 1; i++) {
            double obj = tmpPaths[i].L + p * std::log(tmpPaths[i].D);
            if (obj < vals[i]) { // Keep the better path
                vals[i] = obj;
                paths[i] = tmpPaths[i];
            }
            
            /* for debug
            if (i == 10369) {
                printf("H %f %f dist_con=%f\n", obj, vals[i], _Dijkstra_dist_constraint);
            }
             */
        }
        
        numDijkstra++;
    }
    
    _Dijkstra_dist_constraint = INFINITY;
    
#ifdef SHOW_DEBUG2
    printf("number of Dijkstra=%d\n", numDijkstra);
#endif
    
    delete[] vals;
    delete[] tmpPaths;
    
    return count;
}
