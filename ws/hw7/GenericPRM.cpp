#include "MyHW7.h"
#include "MyHW6.h"
#include "MySampleAlgo.h"
#include "AMPCore.h"

GenericPRM::GenericPRM(int _n, double _r, bool _smoothing)
{
    n = _n;
    r = _r;
    smoothing = _smoothing;
};

amp::Path GenericPRM::plan(const Eigen::VectorXd &init_state,
                           const Eigen::VectorXd &goal_state,
                           const amp::ConfigurationSpace &collision_checker)
{
    LOG("PRM planning...");
    limits.lower_limits = collision_checker.lowerBounds();
    limits.upper_limits = collision_checker.upperBounds();

    findRandomStates(init_state, goal_state, collision_checker);
    LOG("all_states size: "<< all_states.size());
    connectStates(collision_checker);

    MyAStarAlgo::GraphSearchResult Astar_result;
    amp::ShortestPathProblem path_problem;
    path_problem.graph = std::make_unique<amp::Graph<double>>(coord_map);
    //coord_map.print();
    path_problem.init_node = n;
    path_problem.goal_node = n + 1;
    //LOG("start/end nodes: " << path_problem.init_node << " " << path_problem.goal_node);
    MyAStarAlgo Astar;
    amp::SearchHeuristic dijkstra;
    Astar_result = Astar.search(path_problem, dijkstra);

    amp::Path path;
    LOG("Astar complete. Path size: " << Astar_result.node_path.size());
    for (int ith_element : Astar_result.node_path)
    {
        path.waypoints.push_back(all_states[ith_element]);
    }

    return path;
}

void GenericPRM::connectStates(const amp::ConfigurationSpace &collision_checker)
{
    double state_diff_mag;
    node_counter = 0;
    for (int i_observe = 0; i_observe < (n + 2); i_observe++)
    {
        // LOG("Loop i: " << i_observe);
        createNode(all_states[i_observe]);
        for (int i_n = 0; i_n < (n + 2); i_n++)
        {
            if (i_observe == i_n)
            {
                continue;
            }
            state_diff_mag = (all_states[i_observe] - all_states[i_n]).norm();
            if ((state_diff_mag < r) && (!inLineCollision(all_states[i_observe], all_states[i_n], collision_checker)))
            {
                connectNode(i_n);
            }
        }
        node_counter++;
    }
}

void GenericPRM::findRandomStates(const Eigen::VectorXd &init_state,
                                  const Eigen::VectorXd &goal_state,
                                  const amp::ConfigurationSpace &collision_checker)
{
    Eigen::VectorXd rand_q(collision_checker.dimension());
    int i_rand_state = 0;
    //int num_of_nodes = n + 2;
    while (i_rand_state < n)
    {
        rand_q = getRandomState();
        if (!(collision_checker.inCollision(rand_q)))
        {
            all_states.push_back(rand_q);
            i_rand_state++;
        }
    }
    all_states.push_back(init_state);
    all_states.push_back(goal_state);
}

void GenericPRM::createNode(const Eigen::VectorXd &state)
{
    //LOG("node counter: "<<node_counter);
    node_to_coord[node_counter] = state;
    //coord_map.nodes().push_back(node_counter);
}

void GenericPRM::connectNode(const int &i_n)
{
    Eigen::VectorXd observed_state = all_states[node_counter];
    Eigen::VectorXd i_state = all_states[i_n];
    double distance = (observed_state - i_state).norm();
    coord_map.connect(node_counter, i_n, distance);
}

amp::Graph<double> GenericPRM::getCoordMap()
{
    return coord_map;
}

std::map<amp::Node, Eigen::Vector2d> GenericPRM::getNode2Coord()
{
    return node_to_coord;
}

bool GenericPRM::inLineCollision(const Eigen::VectorXd &p1, const Eigen::VectorXd &p2, const amp::ConfigurationSpace &collision_checker) const
{
    // Discretize the line segment
    int numSegments = 100; // Adjust the number of segments as needed for accuracy
    Eigen::VectorXd delta = (p2 - p1) / numSegments;

    for (int ii = 0; ii <= numSegments; ii++)
    {
        Eigen::VectorXd seg_point = (ii * delta) + p1;
        // Check if the point is in the polygon
        if (collision_checker.inCollision(seg_point))
        {
            return true; // Intersection found
        }
    }
    return false; // No intersection found
}

Eigen::VectorXd GenericPRM::getRandomState()
{
    std::size_t num_dim = limits.lower_limits.size();
    Eigen::VectorXd q_state(num_dim);
    // LOG("num dim: " << num_dim);

    for (int i_dim = 0; i_dim < num_dim; i_dim++)
    {
        // LOG("ith dim: " << i_dim);
        // LOG("test lim: " << limits.lower_limits(i_dim));
        // LOG("test lim: " << limits.upper_limits(i_dim));
        double q_i = amp::RNG::randd(limits.lower_limits(i_dim), limits.upper_limits(i_dim));
        // LOG("state: " << q_i);
        q_state.coeffRef(i_dim) = q_i;
    }
    // LOG("state:" << q_state);
    return q_state;
}