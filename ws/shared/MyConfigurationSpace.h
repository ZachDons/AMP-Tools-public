#pragma once // includes are included only once
#include "AMPCore.h"
#include <cmath>
#include <iostream>
#include "MyLinkManipulator2D.h"
using std::vector, Eigen::Vector2d, std::cout;
using namespace amp;

class MyConfigurationSpace : public amp::GridCSpace2D
{
public:
    MyConfigurationSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max);
    bool isPointInPolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &q);
    bool isLineIntersectingPolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);
    //bool isChainIntersectingPolygon(const std::vector<Eigen::Vector2d> &polygon, const MyLinkManipulator2D &robot, const ManipulatorState &state);

    //void compute_Cspace(const vector<Obstacle2D> &obstacles, const MyLinkManipulator2D &robot);
    void compute_Cspace(const vector<Obstacle2D> &obstacles);

    virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;

    // Vector2d getPointFromIndex(const std::pair<int, int> &i_x);

private:
    // member variables
    vector<Obstacle2D> all_obstacles;
    // MyLinkManipulator2D my_robot;
};