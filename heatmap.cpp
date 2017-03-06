//
//  heatmap.cpp
//  DominantPath
//
//  Created by Ger Yang on 2017/3/6.
//  Copyright (c) 2017年 Ger Yang. All rights reserved.
//

#include "floorplan.h"
#include <algorithm>
#include <iostream>

#ifdef USE_OPEN_CV

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define SHOW_DEBUG

cv::Mat myPrintPath(cv::Mat &image, Path &path, double fScale, double fShift) {
    
    cv::Mat image2 = image.clone();
    
    DijkstraPoint ptr1 = path.v[0];
    
    printf("(L=%f, D=%f): ", path.L, path.D);
    printf("%d", ptr1.p->i);
    
    for (int j=1; j < (int) path.v.size(); j++) {
        DijkstraPoint ptr2 = path.v[j];
        cv::Point p1(ptr1.p->x() * fScale + fShift, ptr1.p->y() * fScale + fShift);
        cv::Point p2(ptr2.p->x() * fScale + fShift, ptr2.p->y() * fScale + fShift);
        cv::line(image2, p1, p2, cv::Scalar(255,0,0), 2);
        
        printf(" -> %d", ptr2.p->i);
        
        ptr1 = ptr2;
    }
    
    printf("\n");
    
    return image2;
}

struct myData {
    DominantPath *dmp;
    Path *paths;
    cv::Mat *image;
    int totx;
    int toty;
    int precision;
};

void callbackfunc(int event, int x, int y, int flags, void *data) {
    myData *md = (myData *)data;
    double fScale = md->dmp->fScale;
    double fShift = md->dmp->fShift;
    int precision = md->precision;
    
    int xx = (int)(double(x-fShift) / fScale * precision);
    int yy = (int)(double(y-fShift) / fScale * precision);
    
    if (event == cv::EVENT_LBUTTONDOWN && xx >= 0 && yy >= 0 && xx < md->totx && yy < md->toty){
        printf("(%d,%d)  (%d,%d)", x,y,xx,yy);
        cv::Mat image = myPrintPath(*(md->image), md->paths[xx * md->toty + yy-1], fScale, fShift);
        cv::imshow( "Display window", image );
    } if (event == cv::EVENT_RBUTTONDOWN) {
        cv::imshow( "Display window", *(md->image));
    }
}


void DominantPath::heatmap(double p, double step, int sx, int sy, int x, int y, int precision) {
    
    // First create a set of measurement points
    int totx = x * precision;
    int toty = y * precision;
    
    Point *pts = new Point[totx*toty];
    
    int z = 0;
    for (int i = 0; i < totx; i++) {
        for (int j = 0; j < toty; j++, z++) {
            pts[i * toty + j].x = double(i + 0.5)/precision;
            pts[i * toty + j].y = double(j + 0.5)/precision;
        }
    }
    
    // Set (sx, sy) to be the source
    std::swap( pts[0], pts[sx * toty + sy]);
    
    // Set up the points
    _mPoints = pts;
    _nmPoints = totx * toty;
    
    // Generate G2
    generateG2();
    
#ifdef SHOW_DEBUG
    std::cerr << "G2 generated." << std::endl;
#endif
    
    // Run approx alg
    Path *paths = new Path[totx * toty - 1];
    Approx_all_dest(p, step, paths);
    double max_loss = 0.0;
    for (int i=0; i < totx*toty-1; i++) {
        if (max_loss < paths[i].L + p * std::log(paths[i].D))
            max_loss = paths[i].L + p * std::log(paths[i].D);
    }
    
#ifdef SHOW_DEBUG
    printf("Max loss: %f\n", max_loss);
    std::cerr << "Generating heat map." << std::endl;
#endif
    
    
    cv::Mat image(fSizeX, fSizeY, CV_8UC3, cv::Scalar(255,255,255));
    

    // Draw results
    cv::Point center(pts[0].x * fScale +fShift, pts[0].y * fScale + fShift);
    cv::circle(image, center, 4, cv::Scalar(255,0,0), 2);
    
    for (int i=1; i < totx*toty; i++) {
        double loss = paths[i-1].L + p * std::log(paths[i-1].D);
#ifdef SHOW_DEBUG
        //printf("Node %d (%f,%f)  loss: %f\n", i, pts[i].x, pts[i].y, max_loss);
        //if (i == sx * toty + sy - 2)
#endif
        //cv::Point center(pts[i].x * fScale +fShift, pts[i].y * fScale + fShift);
        cv::Point p1((pts[i].x-0.51/precision) * fScale +fShift, (pts[i].y-0.51/precision) * fScale + fShift);
        cv::Point p2((pts[i].x+0.51/precision) * fScale +fShift, (pts[i].y+0.51/precision) * fScale + fShift);
        cv::Rect2d rect(p1, p2);
        //cv::circle(image, center, 3, cv::Scalar(0,0,int((loss/max_loss)*256)));
        cv::rectangle(image, rect, cv::Scalar(0,0,int((1.1-loss/max_loss)*256)), -1);
    }
    
    // Draw floorplan
    for (int i=0; i < _nG2Corners; i++) {
        CornerG2 *point = &_G2Corners[i];
        cv::Point center(point->x() * fScale +fShift, point->y() * fScale + fShift);
        cv::circle(image, center, 3, cv::Scalar(0,0,0));
    }
    
    for (int i=0; i < _flp->getNumWalls(); i++) {
        Wall *wall = _flp->getWallPtr(i);
        cv::Point p1(wall->c1->x * fScale + fShift, wall->c1->y * fScale + fShift);
        cv::Point p2(wall->c2->x * fScale + fShift, wall->c2->y * fScale + fShift);
        cv::line(image, p1, p2, cv::Scalar(0,0,0), 2);
    }
    
    //myPrintPath(image, paths[sx * toty + sy - 2], fScale, fShift);
    
    myData md;
    md.dmp = this;
    md.paths = paths;
    md.image = &image;
    md.totx = totx;
    md.toty = toty;
    md.precision = precision;
    
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::setMouseCallback("Display window", callbackfunc, &md);
    cv::imshow( "Display window", image );
    cv::waitKey(0);
    
    delete[] pts;
    delete[] paths;
}

#endif