#include "ZachTools.h"

std::pair<std::size_t, std::size_t> ZachTools::getCellFromPoint(const amp::GridCSpace2D &grid_cspace, double x0, double x1) 
{
    // Given point, return cell
    std::pair<int, int> grid_size = grid_cspace.size();
    std::pair<double, double> x0_lim = grid_cspace.x0Bounds();
    std::pair<double, double> x1_lim = grid_cspace.x1Bounds();
    double x0_step = (x0_lim.second - x0_lim.first) / grid_size.first;
    double x1_step = (x1_lim.second - x1_lim.first) / grid_size.second;
    // std::size_t x0_index = static_cast<int>((x0 - x0_lim.first) / (x0_step));
    // std::size_t x1_index = static_cast<int>((x1 - x1_lim.first) / (x1_step));
    std::size_t x0_index = static_cast<int>(std::floor((x0 - x0_lim.first) / (x0_step)));
    std::size_t x1_index = static_cast<int>(std::floor((x1 - x1_lim.first) / (x1_step)));
    LOG("getCell Index: " << x0_index << "," << x1_index);
    //std::cout << "getCell Index: " << x0_index << "," << x1_index << std::endl;
    // std::floor
    return {x0_index, x1_index};
}

Vector2d ZachTools::getPointFromIndex(const amp::GridCSpace2D &grid_cspace,const std::pair<int, int> &i_x)
{
    std::pair<int, int> grid_size = grid_cspace.size();
    std::pair<double, double> x0_lim = grid_cspace.x0Bounds();
    std::pair<double, double> x1_lim = grid_cspace.x1Bounds();
    double q_x0 = ((x0_lim.second - x0_lim.first) / grid_size.first) * i_x.first + x0_lim.first;
    double q_x1 = ((x1_lim.second - x1_lim.first) / grid_size.second) * i_x.second + x1_lim.first;
    return {q_x0, q_x1};
}