// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"
// #include "hw/HW4.h"
// #include "hw/HW5.h"
#include "hw/HW5.h"
#include "MyHW7.h"
#include "MySampleAlgo.h"
// #include "MyConfigurationSpace.h"

using namespace amp;

void problem1a1()
{
    Problem2D problem = HW5::getWorkspace1();
    // problem.y_min = -3;
    // problem.y_max = 3;
    //  std::list<std::vector<double>> data_set;
    //  std::vector<std::string>;
    int n = 200;
    double r = 1;
    bool smoothing = false;
    MyPRM2D algo(n, r, smoothing);
    amp::Path2D path = algo.plan(problem);
    LOG("FINAL PATH LENGTH: " << path.length());
    // algo.getCoordMap().print();
    // LOG("size of getNode2Coord "<<algo.getNode2Coord().nodes().size());
    amp::Visualizer::makeFigure(problem, algo.getCoordMap(), algo.getNode2Coord());
    amp::Visualizer::makeFigure(problem, path);
}

void problem1a2()
{
    Problem2D problem = HW5::getWorkspace1();
    problem.y_min = -3;
    problem.y_max = 3;
    std::vector<std::pair<int, double>> nr_values = {{200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0}, {500, 0.5}, {500, 1.0}, {500, 1.5}, {500, 2.0}};
    std::list<std::vector<double>> path_length_set;
    std::list<std::vector<double>> comp_time_set;
    std::vector<double> valid_soln_set;

    std::vector<std::string> labels = {"{200, 0.5}", "{200, 1}", "{200, 1.5}", "{200, 2}", "{500, 0.5}", "{500, 1}", "{500, 1.5}", "{500, 2}"};
    std::string xlabel = "(n, r) Benchmarks";

    int total_runs = 100;
    amp::Timer timer("t");
    double start_time;
    for (const std::pair<int, double> i_nr : nr_values)
    {
        std::vector<double> path_length;
        std::vector<double> comp_time;
        int valid_soln_counter = 0;
        start_time = timer.now(amp::TimeUnit::ms);
        for (int i_run = 0; i_run < total_runs; i_run++)
        {
            amp::Timer("t");
            int n = i_nr.first;
            double r = i_nr.second;
            bool smoothing = false;
            LOG("n = " << n << ", r = " << r);

            MyPRM2D algo(n, r, smoothing);
            amp::Path2D path = algo.plan(problem);
            // Computation Length
            comp_time.push_back(timer.now(amp::TimeUnit::ms) - start_time);
            // Path length
            if (path.waypoints.size() == 0)
            {
                path_length.push_back(0);
            }
            else
            {
                path_length.push_back(path.length());
                valid_soln_counter++;
            }
        }

        path_length_set.push_back(path_length);
        comp_time_set.push_back(comp_time);
        valid_soln_set.push_back(valid_soln_counter);
    }

    amp::Visualizer::makeBoxPlot(path_length_set, labels,
                                 "Path Length Benchmark", xlabel, "Path lengths, [units]");
    amp::Visualizer::makeBoxPlot(comp_time_set, labels,
                                 "Computation Time Benchmark", xlabel, "Time, [ms]");
    amp::Visualizer::makeBarGraph(valid_soln_set, labels,
                                  "Number of Valid Solutions", xlabel, "Number of valid solutions");
}

void problem1b1()
{
    Problem2D problem = HW2::getWorkspace1();
    int n = 500;
    double r = 2;
    bool smoothing = false;
    MyPRM2D algo(n, r, smoothing);
    amp::Path2D path = algo.plan(problem);
    LOG("FINAL PATH LENGTH: " << path.length());
    // algo.getCoordMap().print();
    // LOG("size of getNode2Coord "<<algo.getNode2Coord().nodes().size());
    amp::Visualizer::makeFigure(problem, algo.getCoordMap(), algo.getNode2Coord());
    amp::Visualizer::makeFigure(problem, path);
}
void problem1b1i()
{
    Problem2D problem = HW2::getWorkspace2();
    int n = 500;
    double r = 2;
    bool smoothing = false;
    MyPRM2D algo(n, r, smoothing);
    amp::Path2D path = algo.plan(problem);
    LOG("FINAL PATH LENGTH: " << path.length());
    // algo.getCoordMap().print();
    // LOG("size of getNode2Coord "<<algo.getNode2Coord().nodes().size());
    amp::Visualizer::makeFigure(problem, algo.getCoordMap(), algo.getNode2Coord());
    amp::Visualizer::makeFigure(problem, path);
}

void problem1b2()
{
    Problem2D problem = HW2::getWorkspace1();
    std::vector<std::pair<int, double>> nr_values = {{200, 1.0}, {200, 2.0}, {500, 1.0}, {500, 2.0}, {1000, 1.0}, {1000, 2.0}};
    std::list<std::vector<double>> path_length_set;
    std::list<std::vector<double>> comp_time_set;
    std::vector<double> valid_soln_set;

    std::vector<std::string> labels = {"{200, 1}", "{200, 2}", "{500, 1}", "{500, 2}", "{1000, 1.0}", "{1000, 2.0}"};
    std::string xlabel = "(n, r) Benchmarks";

    int total_runs = 100;
    amp::Timer timer("t");
    double start_time;
    for (const std::pair<int, double> i_nr : nr_values)
    {
        std::vector<double> path_length;
        std::vector<double> comp_time;
        int valid_soln_counter = 0;
        start_time = timer.now(amp::TimeUnit::ms);
        for (int i_run = 0; i_run < total_runs; i_run++)
        {
            amp::Timer("t");
            int n = i_nr.first;
            double r = i_nr.second;
            bool smoothing = false;
            LOG("n = " << n << ", r = " << r);

            MyPRM2D algo(n, r, smoothing);
            amp::Path2D path = algo.plan(problem);
            // Computation Length
            comp_time.push_back(timer.now(amp::TimeUnit::ms) - start_time);
            // Path length
            if (path.waypoints.size() == 0)
            {
                path_length.push_back(0);
            }
            else
            {
                path_length.push_back(path.length());
                valid_soln_counter++;
            }
        }

        path_length_set.push_back(path_length);
        comp_time_set.push_back(comp_time);
        valid_soln_set.push_back(valid_soln_counter);
    }

    amp::Visualizer::makeBoxPlot(path_length_set, labels,
                                 "Path Length Benchmark", xlabel, "Path lengths, [units]");
    amp::Visualizer::makeBoxPlot(comp_time_set, labels,
                                 "Computation Time Benchmark", xlabel, "Time, [ms]");
    amp::Visualizer::makeBarGraph(valid_soln_set, labels,
                                  "Number of Valid Solutions", xlabel, "Number of valid solutions");
}

void problem1b2i()
{
    Problem2D problem = HW2::getWorkspace2();
    std::vector<std::pair<int, double>> nr_values = {{200, 1.0}, {200, 2.0}, {500, 1.0}, {500, 2.0}, {1000, 1.0}, {1000, 2.0}};
    std::list<std::vector<double>> path_length_set;
    std::list<std::vector<double>> comp_time_set;
    std::vector<double> valid_soln_set;

    std::vector<std::string> labels = {"{200, 1}", "{200, 2}", "{500, 1}", "{500, 2}", "{1000, 1.0}", "{1000, 2.0}"};
    std::string xlabel = "(n, r) Benchmarks";

    int total_runs = 100;
    amp::Timer timer("t");
    double start_time;
    for (const std::pair<int, double> i_nr : nr_values)
    {
        std::vector<double> path_length;
        std::vector<double> comp_time;
        int valid_soln_counter = 0;
        start_time = timer.now(amp::TimeUnit::ms);
        for (int i_run = 0; i_run < total_runs; i_run++)
        {
            amp::Timer("t");
            int n = i_nr.first;
            double r = i_nr.second;
            bool smoothing = false;
            LOG("n = " << n << ", r = " << r);

            MyPRM2D algo(n, r, smoothing);
            amp::Path2D path = algo.plan(problem);
            // Computation Length
            comp_time.push_back(timer.now(amp::TimeUnit::ms) - start_time);
            // Path length
            if (path.waypoints.size() == 0)
            {
                path_length.push_back(0);
            }
            else
            {
                path_length.push_back(path.length());
                valid_soln_counter++;
            }
        }

        path_length_set.push_back(path_length);
        comp_time_set.push_back(comp_time);
        valid_soln_set.push_back(valid_soln_counter);
    }

    amp::Visualizer::makeBoxPlot(path_length_set, labels,
                                 "Path Length Benchmark", xlabel, "Path lengths, [units]");
    amp::Visualizer::makeBoxPlot(comp_time_set, labels,
                                 "Computation Time Benchmark", xlabel, "Time, [ms]");
    amp::Visualizer::makeBarGraph(valid_soln_set, labels,
                                  "Number of Valid Solutions", xlabel, "Number of valid solutions");
}


void problem2a1()
{
    Problem2D problem = HW5::getWorkspace1();
    problem.y_min = -3;
    problem.y_max = 3;
    int n = 5000;
    double r = 1;
    double p_goal = 0.05;
    double epsilon = 0.25;
    bool smoothing = false;
    MyGoalBiasRRT2D algo(n, r, p_goal, epsilon, smoothing);
    amp::Path2D path = algo.plan(problem);
    LOG("FINAL PATH LENGTH: " << path.length());
    algo.getCoordMap().print();
    LOG("Plotting");
    amp::Visualizer::makeFigure(problem, algo.getCoordMap(), algo.getNode2Coord());
    amp::Visualizer::makeFigure(problem, path);
    LOG("Plot finished");
}

void problem2a2()
{
    Problem2D problem = HW5::getWorkspace1();
    problem.y_min = -3;
    problem.y_max = 3;
    // int n = 5000;
    // double r = 0.5;

    std::vector<std::pair<int, double>> nr_values = {{200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0}, {500, 0.5}, {500, 1.0}, {500, 1.5}, {500, 2.0}};
    std::list<std::vector<double>> path_length_set;
    std::list<std::vector<double>> comp_time_set;
    std::vector<double> valid_soln_set;

    std::vector<std::string> labels = {"{200, 0.5}", "{200, 1}", "{200, 1.5}", "{200, 2}", "{500, 0.5}", "{500, 1}", "{500, 1.5}", "{500, 2}"};
    std::string xlabel = "(n, r) Benchmarks";

    int total_runs = 100;
    amp::Timer timer("t");
    double start_time;
    for (const std::pair<int, double> i_nr : nr_values)
    {
        std::vector<double> path_length;
        std::vector<double> comp_time;
        int valid_soln_counter = 0;
        start_time = timer.now(amp::TimeUnit::ms);
        for (int i_run = 0; i_run < total_runs; i_run++)
        {
            amp::Timer("t");
            int n = i_nr.first;
            double r = i_nr.second;
            double p_goal = 0.05;
            double epsilon = 0.25;
            bool smoothing = false;
            LOG("n = " << n << ", r = " << r);
            MyGoalBiasRRT2D algo(n, r, p_goal, epsilon, smoothing);
            amp::Path2D path = algo.plan(problem);
            LOG("Path Computed");
            // Computation Length
            comp_time.push_back(timer.now(amp::TimeUnit::ms) - start_time);
            // Path length
            if (path.waypoints.size() == 0)
            {
                path_length.push_back(0);
            }
            else
            {
                path_length.push_back(path.length());
                valid_soln_counter++;
            }
        }

        path_length_set.push_back(path_length);
        comp_time_set.push_back(comp_time);
        valid_soln_set.push_back(valid_soln_counter);
    }

    amp::Visualizer::makeBoxPlot(path_length_set, labels,
                                 "Path Length Benchmark", xlabel, "Path lengths, [units]");
    amp::Visualizer::makeBoxPlot(comp_time_set, labels,
                                 "Computation Time Benchmark", xlabel, "Time, [ms]");
    amp::Visualizer::makeBarGraph(valid_soln_set, labels,
                                  "Number of Valid Solutions", xlabel, "Number of valid solutions");
}

void problem2b1()
{
    Problem2D problem = HW2::getWorkspace1();
    int n = 5000;
    double r = 0.5;
    double p_goal = 0.05;
    double epsilon = 0.25;
    bool smoothing = false;
    MyGoalBiasRRT2D algo(n, r, p_goal, epsilon, smoothing);
    amp::Path2D path = algo.plan(problem);
    LOG("FINAL PATH LENGTH: " << path.length());
    // algo.getCoordMap().print();
    LOG("Plotting");
    amp::Visualizer::makeFigure(problem, algo.getCoordMap(), algo.getNode2Coord());
    amp::Visualizer::makeFigure(problem, path);
}

void problem2b1i()
{
    Problem2D problem = HW2::getWorkspace2();
    int n = 5000;
    double r = 0.5;
    double p_goal = 0.05;
    double epsilon = 0.25;
    bool smoothing = false;
    MyGoalBiasRRT2D algo(n, r, p_goal, epsilon, smoothing);
    amp::Path2D path = algo.plan(problem);
    LOG("FINAL PATH LENGTH: " << path.length());
    // algo.getCoordMap().print();
    LOG("Plotting");
    amp::Visualizer::makeFigure(problem, algo.getCoordMap(), algo.getNode2Coord());
    amp::Visualizer::makeFigure(problem, path);
}

void problem2b2()
{
    Problem2D problem = HW2::getWorkspace1();
    std::vector<std::pair<int, double>> nr_values = {{200, 1.0}, {200, 2.0}, {500, 1.0}, {500, 2.0}, {1000, 1.0}, {1000, 2.0}};
    std::list<std::vector<double>> path_length_set;
    std::list<std::vector<double>> comp_time_set;
    std::vector<double> valid_soln_set;

    std::vector<std::string> labels = {"{200, 1}", "{200, 2}", "{500, 1}", "{500, 2}", "{1000, 1.0}", "{1000, 2.0}"};
    std::string xlabel = "(n, r) Benchmarks";

    int total_runs = 100;
    amp::Timer timer("t");
    double start_time;
    for (const std::pair<int, double> i_nr : nr_values)
    {
        std::vector<double> path_length;
        std::vector<double> comp_time;
        int valid_soln_counter = 0;
        start_time = timer.now(amp::TimeUnit::ms);
        for (int i_run = 0; i_run < total_runs; i_run++)
        {
            int n = i_nr.first;
            double r = i_nr.second;
            double p_goal = 0.05;
            double epsilon = 0.25;
            bool smoothing = false;
            LOG("n = " << n << ", r = " << r);
            MyGoalBiasRRT2D algo(n, r, p_goal, epsilon, smoothing);
            amp::Path2D path = algo.plan(problem);
            LOG("Path Computed");
            // Computation Length
            comp_time.push_back(timer.now(amp::TimeUnit::ms) - start_time);
            // Path length
            if (path.waypoints.size() == 0)
            {
                path_length.push_back(0);
            }
            else
            {
                path_length.push_back(path.length());
                valid_soln_counter++;
            }
        }

        path_length_set.push_back(path_length);
        comp_time_set.push_back(comp_time);
        valid_soln_set.push_back(valid_soln_counter);
    }

    amp::Visualizer::makeBoxPlot(path_length_set, labels,
                                 "Path Length Benchmark", xlabel, "Path lengths, [units]");
    amp::Visualizer::makeBoxPlot(comp_time_set, labels,
                                 "Computation Time Benchmark", xlabel, "Time, [ms]");
    amp::Visualizer::makeBarGraph(valid_soln_set, labels,
                                  "Number of Valid Solutions", xlabel, "Number of valid solutions");
}


void problem2b2i()
{
    Problem2D problem = HW2::getWorkspace2();
    std::vector<std::pair<int, double>> nr_values = {{200, 1.0}, {200, 2.0}, {500, 1.0}, {500, 2.0}, {1000, 1.0}, {1000, 2.0}};
    std::list<std::vector<double>> path_length_set;
    std::list<std::vector<double>> comp_time_set;
    std::vector<double> valid_soln_set;

    std::vector<std::string> labels = {"{200, 1}", "{200, 2}", "{500, 1}", "{500, 2}", "{1000, 1.0}", "{1000, 2.0}"};
    std::string xlabel = "(n, r) Benchmarks";

    int total_runs = 100;
    amp::Timer timer("t");
    double start_time;
    for (const std::pair<int, double> i_nr : nr_values)
    {
        std::vector<double> path_length;
        std::vector<double> comp_time;
        int valid_soln_counter = 0;
        start_time = timer.now(amp::TimeUnit::ms);
        for (int i_run = 0; i_run < total_runs; i_run++)
        {
            int n = i_nr.first;
            double r = i_nr.second;
            double p_goal = 0.05;
            double epsilon = 0.25;
            bool smoothing = false;
            LOG("n = " << n << ", r = " << r);
            MyGoalBiasRRT2D algo(n, r, p_goal, epsilon, smoothing);
            amp::Path2D path = algo.plan(problem);
            LOG("Path Computed");
            // Computation Length
            comp_time.push_back(timer.now(amp::TimeUnit::ms) - start_time);
            // Path length
            if (path.waypoints.size() == 0)
            {
                path_length.push_back(0);
            }
            else
            {
                path_length.push_back(path.length());
                valid_soln_counter++;
            }
        }

        path_length_set.push_back(path_length);
        comp_time_set.push_back(comp_time);
        valid_soln_set.push_back(valid_soln_counter);
    }

    amp::Visualizer::makeBoxPlot(path_length_set, labels,
                                 "Path Length Benchmark", xlabel, "Path lengths, [units]");
    amp::Visualizer::makeBoxPlot(comp_time_set, labels,
                                 "Computation Time Benchmark", xlabel, "Time, [ms]");
    amp::Visualizer::makeBarGraph(valid_soln_set, labels,
                                  "Number of Valid Solutions", xlabel, "Number of valid solutions");
}

int main(int argc, char **argv)
{
    // amp::HW7::hint();
    // problem1a1();
    // problem1a2();

    // problem1b1();
    // problem1b1i();

    //  problem1b2();
    // problem1b2i();


    // problem2a1();
    // problem2a2();

    // problem2b1();
    // problem2b1i();

    // problem2b2();
    problem2b2i();


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

    Visualizer::showFigures();
    // amp::RNG::seed(amp::RNG::randiUnbounded());
    //amp::HW7::grade<MyPRM2D, MyGoalBiasRRT2D> ("zachary.donovan@colorado.edu",argc,argv,std::make_tuple(500,2,false),std::make_tuple(5000,0.5,0.05,0.25,false));

    // Grade function
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    //amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("zachary.donovan@colorado.edu", argc, argv);

    return 0;
}

// Questions
// Do we need to use "PointAgent2D" from AgentTypes.h?