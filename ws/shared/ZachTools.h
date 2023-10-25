#pragma once
#include "AMPCore.h"
using std::vector, Eigen::Vector2d;

class ZachTools {
    public:
        static std::pair<std::size_t, std::size_t> getCellFromPoint(const amp::GridCSpace2D &grid_cspace, double x0, double x1);
        static Vector2d getPointFromIndex(const amp::GridCSpace2D &grid_cspace,const std::pair<int, int> &i_x);

};