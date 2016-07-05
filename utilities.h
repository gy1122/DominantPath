//
//  utilities.h
//  DominantPath
//
//  Created by Ger Yang on 2016/6/30.
//  Copyright (c) 2016年 Ger Yang. All rights reserved.
//

#ifndef __DominantPath__utilities__
#define __DominantPath__utilities__


class BinaryHeap {
    
    class BinaryHeapNode {
    public:
        BinaryHeapNode(double w = 0.0);
        virtual bool operator<(const BinaryHeapNode &rhs);
        virtual void setVal(double w);
        
        inline BinaryHeapNode *left() { return _left; }
        inline BinaryHeapNode *right() { return _right; }
        
    private:
        double  _weight;
        BinaryHeapNode *_left;
        BinaryHeapNode *_right;
    };
    
public:
    
private:
    
};

#endif /* defined(__DominantPath__utilities__) */
