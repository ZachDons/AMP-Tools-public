// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"
// #include "hw/HW4.h"
// #include "hw/HW5.h"
#include "hw/HW6.h"
#include "MyHW6.h"
#include "MyConfigurationSpace.h"

using namespace amp;

void problem1a()
{
    Problem2D problem = HW2::getWorkspace1();
    MyPointWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> config_space = algo.constructDiscretizedWorkspace(problem);
    amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *config_space);
    Visualizer::makeFigure(*config_space);
    Visualizer::makeFigure(problem, path);
}

void problem1b()
{
    Problem2D problem = HW2::getWorkspace2();
    MyPointWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> config_space = algo.constructDiscretizedWorkspace(problem);
    amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *config_space);
    Visualizer::makeFigure(problem, path);
    Visualizer::makeFigure(*config_space);
}
void problem2a()
{
    Problem2D problem = HW2::getWorkspace1();
    MyPointWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> config_space = algo.constructDiscretizedWorkspace(problem);
    amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *config_space);
    Visualizer::makeFigure(*config_space);
    Visualizer::makeFigure(problem, path);
}

void problem2b()
{
    Problem2D problem = HW2::getWorkspace2();
    MyPointWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> config_space = algo.constructDiscretizedWorkspace(problem);
    amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *config_space);
    Visualizer::makeFigure(problem, path);
    Visualizer::makeFigure(*config_space);
}

void problem3a()
{
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    SearchHeuristic search_heuristic;
    MyAStarAlgo algo;
    AStar::GraphSearchResult graph_result = algo.search(problem, heuristic);
    //HW6::generateAndCheck(algo, graph_result, problem, true, 0u);
    HW6::generateAndCheck(algo);

}

void problem3b()
{
    ShortestPathProblem problem = HW6::getEx3SPP();
    //LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    SearchHeuristic search_heuristic;
    MyAStarAlgo algo;
    AStar::GraphSearchResult graph_result = algo.search(problem, search_heuristic);
    //HW6::generateAndCheck(algo, graph_result, problem, true, 0u);
}

int main(int argc, char **argv)
{
    // problem1a();
    // problem1b();
    //   problem2();
    //problem3a();
    //problem3b();

    // Visualizer::showFigures();
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    // Grade function
    //amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("zachary.donovan@colorado.edu", argc, argv);

    return 0;
}