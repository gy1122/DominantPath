#ifndef __DominantPath__wall_finder__
#define __DominantPath__wall_finder__

#include "floorplan.h"
#include "gridder.h"
#include <algorithm>
#include <vector>

class WallFinder {
public:
    explicit WallFinder(const Floorplan& flp, int grid_dim = 50,
                        double epsilon = 1e-6) :
        grid_dim_(grid_dim),
        scale_(grid_dim / std::max(flp.getWidth(), flp.getHeight())),
        gridder_(scale_, epsilon),
        grid_walls_((grid_dim+4)*(grid_dim+4)) {
        for (int i=0; i < flp.getNumWalls(); ++i) {
            const Wall* wall = flp.getWallPtr(i);
            for (const std::pair<int,int>& cell : gridder_.segment(
                     wall->c1->x, wall->c1->y, wall->c2->x, wall->c2->y)) {
                grid_walls_[cell_index(cell)].push_back(wall);
            }
        }
    }
    std::vector<const Wall*> getWalls (double x0, double y0, double x1,
                                       double y1) const {
        std::vector<const Wall*> walls;
        for (const std::pair<int,int>& cell : gridder_.segment(x0, y0, x1, y1)) {
            assert(cell.first >= -2 && cell.first <= grid_dim_+1);
            assert(cell.second >= -2 && cell.second <= grid_dim_+1);
            const std::vector<const Wall*>& wall_list = grid_walls_[cell_index(cell)];
            walls.insert(walls.end(), wall_list.begin(), wall_list.end());
        }
        std::sort(walls.begin(), walls.end());
        walls.erase(std::unique(walls.begin(), walls.end()), walls.end());
        return walls;
    }

private:
    int cell_index(const std::pair<int,int>& cell) const {
        return (cell.first + 2) * (grid_dim_ + 4) + (cell.second + 2);
    }
    int grid_dim_;
    double scale_;
    Gridder gridder_;
    typedef std::vector<std::vector<const Wall*>> GridMap;
    GridMap grid_walls_;
};

#endif  // __DominantPath__wall_finder__
