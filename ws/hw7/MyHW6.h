#pragma once // includes are included only once
#include "AMPCore.h"
#include "hw/HW6.h"
#include "MyLinkManipulator2D.h"
#include "MyConfigurationSpace.h"
#include "ZachTools.h"
#include <cmath>
#include <algorithm>
#include <iostream>
using std::cout, std::vector, Eigen::Vector2d;

struct MyElem
{
    int value;
    std::pair<int, int> index;
    std::pair<int, int> parent;
};

struct MyNode
{
    amp::Node nodeID;
    amp::Node parent;
    double pathCost;
    double priority;
};

class MyGridCSpace : public amp::GridCSpace2D
{
public:
    // MyGridCSpace()
    //     : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0) {}
    MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
        : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {}

    virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;
};

// class MyCSpaceCtor : public amp::GridCSpace2DConstructor
// {
// public:
//     virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D &manipulator, const amp::Environment2D &env) override;
//     {
//         return std::make_unique<MyGridCSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
//     }
// };

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm
{
public:
    virtual amp::Path2D planInCSpace(const Eigen::Vector2d &q_init, const Eigen::Vector2d &q_goal, const amp::GridCSpace2D &grid_cspace) override;

    virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D &environment) override
    {
        // std::size_t cells = 40;
        double cell_width = 0.25;
        int x_cells = static_cast<int>(std::floor(environment.x_max - environment.x_min) / cell_width);
        int y_cells = static_cast<int>(std::floor(environment.y_max - environment.y_min) / cell_width);
        std::unique_ptr<MyConfigurationSpace> c_space = std::make_unique<MyConfigurationSpace>(x_cells, y_cells, environment.x_min, environment.x_max, environment.y_min, environment.y_max);
        c_space->compute_Cspace(environment.obstacles); //
        return c_space;
    }

    // // This is just to get grade to work, you DO NOT need to override this method
    // virtual amp::Path2D plan(const amp::Problem2D &problem) override
    // {
    //     return amp::Path2D();
    // }

    bool isPointInPolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &q);
    void makeMyGridSpace(const amp::GridCSpace2D &grid_cspace, DenseArray2D<MyElem> &wave_grid);
    void bufferObstacles(const amp::GridCSpace2D &grid_cspace, DenseArray2D<MyElem> &wave_grid, const int i_row, const int j_col);
    void expandGridSpace(DenseArray2D<MyElem> &wave_grid, const MyElem cell);
    amp::Path2D compilePath(const amp::GridCSpace2D &grid_cspace, const DenseArray2D<MyElem> &wave_grid);

private:
    std::pair<std::size_t, std::size_t> init_cell;
    std::pair<std::size_t, std::size_t> goal_cell;
    bool goal_is_reached;
    std::vector<MyElem> cell_stack;
};

class MyCSpaceCtor : public amp::GridCSpace2DConstructor
{
public:
    virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D &manipulator, const amp::Environment2D &env) override
    {
        return std::make_unique<MyConfigurationSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
    }
};

// class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm
// {
// public:
//     // Default ctor
//     MyManipWFAlgo()
//         : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {}

//     // You can have custom ctor params for all of these classes
//     MyManipWFAlgo(const std::string &beep)
//         : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) { LOG("construcing... " << beep); }

//     // This is just to get grade to work, you DO NOT need to override this method
//     virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D &link_manipulator_agent, const amp::Problem2D &problem) override
//     {
//         return amp::ManipulatorTrajectory2Link();
//     }

//     // You need to implement here
//     virtual amp::Path2D planInCSpace(const Eigen::Vector2d &q_init, const Eigen::Vector2d &q_goal, const amp::GridCSpace2D &grid_cspace) override
//     {
//         return amp::Path2D();
//     }
// };

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm
{
public:
    // Default ctor
    MyManipWFAlgo()
        : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {}

    // You need to implement here
    virtual amp::Path2D planInCSpace(const Eigen::Vector2d &q_init, const Eigen::Vector2d &q_goal, const amp::GridCSpace2D &grid_cspace) override
    {
        return amp::Path2D();
    }
};

class MyAStarAlgo : public amp::AStar
{
public:
    virtual GraphSearchResult search(const amp::ShortestPathProblem &problem, const amp::SearchHeuristic &heuristic) override;
    // bool compareByPriority(const MyNode &a, const MyNode &b);
    // void sortOpenList(const vector<MyNode> &open_list);
    void sortOpenList(std::vector<MyNode> &nodes);
    bool isNodeInList(const MyNode &achild, const std::vector<MyNode> &alist);
    MyNode findNodeByID(const std::vector<MyNode> &closed_list, const amp::Node &targetNodeID);
    void removeNodeByID(std::vector<MyNode> &alist, const amp::Node &targetNodeID);

private:
    vector<MyNode> open_list;
    vector<MyNode> closed_list;
    double best_priority;
};

// // using namespace amp;
// class MyPointWaveFrontAlgo : public amp::PointWaveFrontAlgorithm
// {
// public:
//     // virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) = 0;
// private:
// };

// // using namespace amp;
// class MyManipulatorWaveFrontAlgo : public amp::ManipulatorWaveFrontAlgorithm
// {
// public:
//     // virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) = 0;
// private:
// };

// manipulator we are using planInCSpace
// point: override constructDiscretizedWorkspace
// give yourself a buffer around your obstacle. Bloat your obstacle
// need to develop a point within a polygon check
// difference between astar and D is if your heurisitic is nonzero

// Astar
//  Need to be able to deterimine if a path exists (true/false), path cost, path itself.
//  how to make a search tree correctly?

// get cell from getCellFromPoint
