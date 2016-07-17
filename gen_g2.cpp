//
//  gen_g2.cpp
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
//  Class PointG2
// -------------------------------------

// This function searches the index of the output slot with minimum diffraction loss given input slot
//  input - angle: input angle, should be in [-pi, pi]
//  output:        the index of the slot
// ---------------------------------------------
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


// -------------------------------------
//  Class CornerG2
// -------------------------------------

// Insert wall to the corner
// ---------------------------------------------
void CornerG2::insertSide(WallG2 wall) {
    walls.push_back(wall);
    for (int i=(int)walls.size()-1; i>0; i--) {
        if (walls[i].angle < walls[i-1].angle)
            std::swap(walls[i], walls[i-1]);
    }
}


// -------------------------------------
//  Class DominantPath (constructor & destructor)
// -------------------------------------

DominantPath::DominantPath(Floorplan *flp, int nmpt, Point *mpts, double angleLoss):
_flp(flp), _nmPoints(nmpt), _mPoints(mpts),
_nG2Points(0), _nG2Corners(0), _G2Points(0), _G2Corners(0),
_nG2totPoints(0), _G2totPoints(0), _angleLoss(angleLoss) {
    
}

DominantPath::~DominantPath() {
    if (_G2Points) delete [] _G2Points;
}

// This comparing function is used when sorting the links in a corner
// -------------------------------------
bool mycmp_edge(EdgeG2 &edge1, EdgeG2 &edge2) {
    return (edge1.angle < edge2.angle) || (edge1.angle == edge2.angle && edge1.isAlongWall < edge2.isAlongWall);
}

// -------------------------------------
//  Class DominantPath (constructing G2)
// -------------------------------------

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
    
    // This saves a temporary list of links and we'll sort this later
    EdgeG2 *tmpList = new EdgeG2[numLinks];
    
    int tmpIdx = 0;
    
    // There should be some links generated by points with smaller index
    //   in the beginning.
    // Copy these links in the list to the tmp list, we will process these later.
    for (int j=0; j < corner->links.size(); j++) {
        tmpList[tmpIdx++] = corner->links[j];
    }
    
    // First, create links along the walls
    for (int j=0; j < nAdjWalls; j++) {
        WallG2 wall = corner->walls[j];
        CornerG2 *target = &_G2Corners[wall.to->i];
        
        // Only create link to point with larger index
        if (target->i < corner->i) continue;
        
        // For each wall, create one link on each side
        EdgeG2 edge1, edge2;
        
        edge1.target = edge2.target = target;
        
        edge1.section = j;
        edge2.section = (j+1) % nAdjWalls;
        edge1.isAlongWall = 1;
        edge2.isAlongWall = 2;
        
        // isAlongWall:
        //    1
        //   ------------->
        //    2
        
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
    
    // mergeLinks will remove duplicated links in tmpList
    //  and save the links to corner->links.
    mergeLinks(corner, numLinks, tmpList);
    
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
}

void DominantPath::createLinksForPoint(PointG2 *point) {
    
    int numLinks = _nG2totPoints - 1;
    
    // This saves a temporary list of links
    //  - we'll sort this later
    EdgeG2 *tmpList = new EdgeG2[numLinks];
    
    int tmpIdx = 0;
    
    // There should be some links generated by points with smaller index
    //   in the beginning.
    // Copy these links in the list to the tmp list, we will process these later.
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
    
    // mergeLinks will remove duplicated links in tmpList
    //  and save the links to corner->links.
    mergeLinks(point, numLinks, tmpList);
    
    createLinksPostProc(point);
    
    delete [] tmpList;
}

void DominantPath::mergeLinks(PointG2 *point, int nEdges, EdgeG2 *tmpList) {
    
    // In this function, we remove duplicated links in tmpList and save the links to point->links.
    // -------------------------------------
    
    // This list keeps tracks of which link to be kept.
    bool *tmpListKeep = new bool[nEdges];
    for (int j=0; j < nEdges; j++) tmpListKeep[j] = false;
    
    // Sort the list and remove duplicated links
    std::sort(tmpList, tmpList + nEdges, mycmp_edge);
    
    // j is the idx to keep
    int first = -1;
    for (int j=0, k=1;;k++) {
        
        // Terminating condition
        if (k == nEdges) {
            
            // Handle +pi/-pi
            if (tmpList[j].target == tmpList[first].target && j != first) {
                
                // Keep the one that is along the wall
                if (tmpList[j].isAlongWall) {
                    tmpListKeep[j] = true;
                    tmpListKeep[first] = false;
                }
            } else {
                tmpListKeep[j] = true;
            }
            break;
        }
        
        //
        if (std::abs(tmpList[j].angle - tmpList[k].angle) > 1.0E-6) {
            
            // If two lines have different angle, keep j.
            
            tmpListKeep[j] = true;
            if (first < 0) first = j;
            j=k;
            
        } else {
            
            // If two lines have the same angle
            // Intuitively, we should keep the line with smaller dist
            // However, in any way we should keep the lines along a wall
            
            // Remove duplicated links
            // Duplicated link should happen only when the link goes along a wall.  Thus we can just remove the link that is not generated in the first step.
            
            if (tmpList[j].target == tmpList[k].target) {
                if (tmpList[k].isAlongWall == 0) continue;
                if (tmpList[j].isAlongWall == 0) {
                    // don't keep j
                    j=k; continue;
                }
            }
            
            // If k is shorter than j, we should discard j.
            // Exceptions:
            //  - If j goes along a wall, then k should be a measurement point.  In this case, we keep both.  But we should prevent this from happening when choosing measurement points.
            //  - If j is created by another point with smaller index, this should be a result of numerical issues.  Since things gets complicated if we remove j, we keep j in this case.
            
            if (tmpList[k].dist < tmpList[j].dist) {
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
        
        if (tmpListKeep[j])
            point->links.push_back(tmpList[j]);
        
        // Make sure we have not missed these musts
        if (tmpList[j].isAlongWall || tmpList[j].target->i < point->i)
            assert(tmpListKeep[j]);
    }
    
    delete [] tmpListKeep;
}

void DominantPath::createLinksPostProc(PointG2 *point) {
    
    // In this function, we do the following things:
    //  1.   Find loss for each link
    //  2-1. Index each link (filling in EdgeG2::source_i
    //  2-2. Fill in EdgeG2::target_i for reverse links from points with smaller index
    //  3.   Create reverse links for points with larger index
    // -------------------------------------
    
    for (int j=0; j < point->links.size(); j++) {
        PointG2 *target = point->links[j].target;
        
        // Find loss
        if (point->links[j].isAlongWall == 0 && target->i > point->i) {
            point->links[j].loss = getLoss(point, point->links[j]);
        }
        
        // Fill in the index
        point->links[j].source_i = j;
        
        // Fill in the index for reverse link
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
            
            // If the link goes along a wall, we need to compute which sector it is in more carefully.
            
            // Idea:
            //   1           2
            //   ------------>
            //   <------------
            //   2           1
            
            if (edge.isAlongWall) {
                int wallId;
                if (edge.isAlongWall == 1) {
                    wallId = target_corner->wdecr(sec);
                    edge.isAlongWall = 2; // revert this
                } else {
                    wallId = sec;
                    edge.isAlongWall = 1;
                }
                edge.target = point;
                
                // The angle of the link along the wall should be always set to the angle of the wall
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
            // Maintain loss & dist
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


// -------------------------------------
//  Class DominantPath (helper functions)
// -------------------------------------

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
    
    // clockwise
    for (int i=insec; i != outsec; i = corner->wincr(i)) {
        loss1 += corner->walls[i].loss;
    }
    
    // counterclockwise
    for (int i=insec; i != outsec; i = corner->wdecr(i)) {
        loss2 += corner->walls[corner->wdecr(i)].loss;
    }
    
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




