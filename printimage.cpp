//
//  printimage.cpp
//  DominantPath
//
//  Created by Ger Yang on 2016/7/25.
//  Copyright (c) 2016年 Ger Yang. All rights reserved.
//

#include "floorplan.h"

#ifdef USE_OPEN_CV

#include <opencv2/highgui/highgui.hpp>

#define SHOW_DEBUG

void DominantPath::makeFloorplanImage(cv::Mat &image) {

    for (int i=0; i < _nG2totPoints; i++) {
        PointG2 *point = _G2totPoints[i];
        cv::Point center(point->x() * fScale +fShift, point->y() * fScale + fShift);
        cv::circle(image, center, 3, cv::Scalar(0,0,0));

        cv::Point font_center(center.x + 2.5, center.y + 12.5);
        cv::putText(image, std::to_string(i), font_center, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, cv::Scalar(0,0,255));
    }

    for (int i=0; i < _flp->getNumWalls(); i++) {
        Wall *wall = _flp->getWallPtr(i);
        cv::Point p1(wall->c1->x * fScale + fShift, wall->c1->y * fScale + fShift);
        cv::Point p2(wall->c2->x * fScale + fShift, wall->c2->y * fScale + fShift);
        cv::line(image, p1, p2, cv::Scalar(0,0,0), 2);
    }

}

void DominantPath::printFloorplan() {
    cv::Mat image(fSizeX, fSizeY, CV_8UC3, cv::Scalar(255,255,255));
    makeFloorplanImage(image);

    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", image );
    cv::waitKey(0);
}

// Check if the data structure is correct
void DominantPath::printG2(int p) {

    cv::Mat image(fSizeX, fSizeY, CV_8UC3, cv::Scalar(255,255,255));
    makeFloorplanImage(image);

    int j = p;
    PointG2 *point = _G2totPoints[j];
    cv::Point center(point->x() * fScale + fShift, point->y() * fScale + fShift);
    for (int i=0; i < (int) point->links.size(); i++) {
        EdgeG2 edge = point->links[i];
        double x = point->x() + edge.dist * std::cos(edge.angle);
        double y = point->y() + edge.dist * std::sin(edge.angle);
        cv::Point p2(x * fScale + fShift, y * fScale + fShift);
        cv::line(image, center, p2, cv::Scalar(255,0,0));
    }

#ifdef SHOW_DEBUG
    // For debugging
    if (point->isCorner()) {
        CornerG2 *corner = reinterpret_cast<CornerG2 *>(point);
        printf("--Adjacent Walls--\n   Angle, Loss\n");
        for (int i=0; i < (int) corner->walls.size(); i++) {
            printf("%+.5f, %.3f\n", corner->walls[i].angle, corner->walls[i].loss);
        }
    }

    printf("\nTotal links: %d\n", (int)point->links.size());
    printf("   Angle, Sec,     Dist,  Loss\n");
    for (int i=0; i < (int) point->links.size(); i++) {
        printf("%+.5f, %3d, % 3.5f, %.3e, %d\n", point->links[i].angle, point->links[i].section, point->links[i].dist, point->links[i].loss, point->links[i].isAlongWall);
    }

#endif

    namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", image );
    cv::waitKey(0);
}

void DominantPath::printPaths(int npaths, Path *paths) {

    cv::Mat image(fSizeX, fSizeY, CV_8UC3, cv::Scalar(255,255,255));
    makeFloorplanImage(image);


    printf("Total: %d paths.\n", npaths);

    for (int i=0; i < npaths; i++) {
        cv::Mat image2 = image.clone();

        DijkstraPoint ptr1 = paths[i].v[0];

        printf("(L=%f, D=%f): ", paths[i].L, paths[i].D);
        printf("%d", ptr1.p->i);

        for (int j=1; j < (int) paths[i].v.size(); j++) {
            DijkstraPoint ptr2 = paths[i].v[j];
            cv::Point p1(ptr1.p->x() * fScale + fShift, ptr1.p->y() * fScale + fShift);
            cv::Point p2(ptr2.p->x() * fScale + fShift, ptr2.p->y() * fScale + fShift);
            cv::line(image2, p1, p2, cv::Scalar(255,0,0), 2);

            printf(" -> %d", ptr2.p->i);

            ptr1 = ptr2;
        }

        printf("\n");

        std::string str = "Press any key to continue.";
        cv::putText(image2, str, cv::Point(300,550), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));

        namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
        cv::imshow( "Display window", image2 );
        cv::waitKey(0);
    }

}

#endif
