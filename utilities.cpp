//
//  utilities.cpp
//  DominantPath
//
//  Created by Ger Yang on 2016/6/30.
//  Copyright (c) 2016年 Ger Yang. All rights reserved.
//

#include "utilities.h"

BinaryHeap::BinaryHeapNode::BinaryHeapNode(double w):_weight(w), _left(0), _right(0) {

}

bool BinaryHeap::BinaryHeapNode::operator<(const BinaryHeapNode &rhs) {
    return (_weight<rhs._weight);
}

void BinaryHeap::BinaryHeapNode::setVal(double w) {
    _weight = w;
}