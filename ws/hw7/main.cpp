// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
// #include "hw/HW2.h"
// #include "hw/HW4.h"
// #include "hw/HW5.h"
#include "hw/HW5.h"
#include "MyHW7.h"
#include "MyConfigurationSpace.h"

using namespace amp;

void problem1a()
{
    Problem2D problem = HW5::getWorkspace1();
    MyPRM2D algo(100, 1);
    amp::Path2D path = algo.plan(problem);
    //amp::Visualizer::makeFigure(problem, coord_map, node_to_coord);
}

// void problem1b()
// {
//     Problem2D problem = HW2::getWorkspace1();

//     MyPointWFAlgo algo;
//     std::unique_ptr<amp::GridCSpace2D> config_space = algo.constructDiscretizedWorkspace(problem);
//     amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *config_space);
//     Visualizer::makeFigure(problem, path);
//     Visualizer::makeFigure(*config_space);
// }

int main(int argc, char **argv)
{
    problem1a();
    //amp::HW7::hint();
    // problem1b();
    // problem2a();
    // problem2b();

    // amp::Graph<double> coord_map;
    // coord_map.connect(0,1,1.0);
    // coord_map.connect(1,2,1.0);
    // coord_map.connect(1,3,1.0);
    // std::map<amp::Node, Eigen::Vector2d> node_to_coord;
    // node_to_coord[0] = Eigen::Vector2d(3.5,4.0);
    // node_to_coord[1] = Eigen::Vector2d(4.5,4.0);
    // node_to_coord[2] = Eigen::Vector2d(5.5,4.0);
    // amp::Visualizer::makeFigure(problem, coord_map, node_to_coord);
    // // amp::Visualizer::makeFigure(const Problem2D& prob, const Graph<double>& map, const FXN& getCoordinateFromNode);
    // // amp::Visualizer::makeFigure(const Problem2D& prob, const Graph<double>& coordinate_map, const std::map<amp::Node, Eigen::Vector2d>& node_to_coordinate);

    // Visualizer::showFigures();
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    // Grade function
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("zachary.donovan@colorado.edu", argc, argv);

    return 0;
}

// Questions
// Do we need to use "PointAgent2D" from AgentTypes.h?