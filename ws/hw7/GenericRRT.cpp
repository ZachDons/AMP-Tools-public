#include "MyHW7.h"
#include "MyHW6.h"
#include "MySampleAlgo.h"
#include "AMPCore.h"

GenericRRT::GenericRRT(int _n, double _r, double _p_goal, double _epsilon, bool _smoothing)
{
    n = _n;
    r = _r;
    p_goal = _p_goal;
    epsilon = _epsilon;
    smoothing = _smoothing;
};

amp::Path GenericRRT::plan(const Eigen::VectorXd &init_state,
                           const Eigen::VectorXd &goal_state,
                           const amp::ConfigurationSpace &collision_checker)
{
    LOG("RRT planning...");
    limits.lower_limits = collision_checker.lowerBounds();
    limits.upper_limits = collision_checker.upperBounds();

    Eigen::VectorXd q_rand;
    std::pair<amp::Node, Eigen::VectorXd> q_nearest;
    Eigen::VectorXd q_new;
    double distance_new2goal;
    double rand_p_goal;

    bool goal_is_reached = false;
    amp::Node node_counter = 0;
    createNode(init_state, node_counter);
    while ((!goal_is_reached) && (node_counter < n))
    {
        // LOG("node_counter: " << node_counter);
        rand_p_goal = amp::RNG::randd(0, 1);
        if (rand_p_goal < p_goal)
        {
            q_rand = goal_state;
        }
        else
        {
            q_rand = getRandomState();
        }

        q_nearest = getNearestToRand(q_rand, node_counter);
        q_new = getNewState(q_nearest.second, q_rand);
        if (!inLineCollision(q_nearest.second, q_new, collision_checker))
        {
            createNode(q_new, node_counter);
            connectNode(q_nearest, q_new, node_counter);
            distance_new2goal = (goal_state - q_new).norm();
            // LOG("distance_new2goal: " << distance_new2goal);
            if (distance_new2goal < epsilon)
            {
                LOG("GOAL REACHED!");
                std::cout<<"GOAL REACHED!"<<std::endl;
                // createNode(goal_state, node_counter);
                node_to_coord[node_counter] = goal_state;
                coord_map.connect(node_counter - 1, node_counter, distance_new2goal);
                std::cout<<"Start:"<< node_to_coord[0] <<std::endl;
                std::cout<<"Goal:"<< node_to_coord[node_counter] <<std::endl;
                std::cout<<"Node Cnt:"<< node_counter <<std::endl;
                goal_is_reached = true;
            }
        }
    }

    LOG("num nodes in coord map " << coord_map.nodes().size());

    amp::Path path;
    if (goal_is_reached)
    {
        path.waypoints.push_back(goal_state);
        vector<amp::Node> parent_node = coord_map.parents(node_counter);
        while (parent_node[0] != 0)
        {
            // LOG("parent_node: " << parent_node[0]);
            //path.waypoints.push_back(node_to_coord[parent_node[0]]);
            path.waypoints.insert(path.waypoints.begin(), node_to_coord[parent_node[0]]);
            parent_node = coord_map.parents(parent_node[0]);
        }
        path.waypoints.insert(path.waypoints.begin(), init_state);
    }
    return path;
}

// void GenericRRT::connectStates(const amp::ConfigurationSpace &collision_checker)
// {
//     double state_diff_mag;
//     node_counter = 0;
//     for (int i_observe = 0; i_observe < (n + 2); i_observe++)
//     {
//         // LOG("Loop i: " << i_observe);
//         createNode(all_states[i_observe]);
//         for (int i_n = 0; i_n < (n + 2); i_n++)
//         {
//             if (i_observe == i_n)
//             {
//                 continue;
//             }
//             state_diff_mag = (all_states[i_observe] - all_states[i_n]).norm();
//             if ((state_diff_mag < r) && (!inLineCollision(all_states[i_observe], all_states[i_n], collision_checker)))
//             {
//                 connectNode(i_n);
//             }
//         }
//         node_counter++;
//     }
// }

// Eigen::VectorXd GenericRRT::findNewState(const Eigen::VectorXd &init_state,
//                                          const Eigen::VectorXd &goal_state,
//                                          const amp::ConfigurationSpace &collision_checker)
// {
//     // Pick a random state
//     Eigen::VectorXd rand_q = findRandomState(const Eigen::VectorXd &init_state,
//                                              const Eigen::VectorXd &goal_state,
//                                              const amp::ConfigurationSpace &collision_checker);
//     // find current nearest member
//     for ()
//     {
//     }

//     int i_rand_state = 0;
//     // int num_of_nodes = n + 2;
//     while (i_rand_state < n)
//     {
//         rand_q = getRandomState();
//         if (!(collision_checker.inCollision(rand_q)))
//         {
//             all_states.push_back(rand_q);
//             i_rand_state++;
//         }
//     }
//     all_states.push_back(init_state);
//     all_states.push_back(goal_state);
// }

// void GenericRRT::findRandomState(const Eigen::VectorXd &init_state,
//                                  const Eigen::VectorXd &goal_state,
//                                  const amp::ConfigurationSpace &collision_checker)
// {
//     Eigen::VectorXd rand_q(collision_checker.dimension());
//     int i_rand_state = 0;
//     // int num_of_nodes = n + 2;
//     while (i_rand_state < n)
//     {
//         rand_q = getRandomState();
//         if (!(collision_checker.inCollision(rand_q)))
//         {
//             all_states.push_back(rand_q);
//             i_rand_state++;
//         }
//     }
//     all_states.push_back(init_state);
//     all_states.push_back(goal_state);
// }

void GenericRRT::createNode(const Eigen::VectorXd &state, amp::Node &node_counter)
{
    // LOG("added node state: " << state);
    node_to_coord[node_counter] = state;
    // coord_map.nodes().push_back(node_counter);
    node_counter++;
}

void GenericRRT::connectNode(const std::pair<amp::Node, Eigen::VectorXd> &q_nearest, const Eigen::VectorXd &q_new, const amp::Node &node_counter)
{
    double distance = (q_new - q_nearest.second).norm();
    // LOG("Connecting: Node " << q_nearest.first << " to " << node_counter - 1);
    coord_map.connect(q_nearest.first, node_counter - 1, distance);
}

amp::Graph<double> GenericRRT::getCoordMap()
{
    return coord_map;
}

std::map<amp::Node, Eigen::Vector2d> GenericRRT::getNode2Coord()
{
    return node_to_coord;
}

// bool GenericRRT::inSubPathCollision(const Eigen::VectorXd &q_nearest, const Eigen::VectorXd &q_rand, const amp::ConfigurationSpace &collision_checker) const
// {
//     double delta_q_mag = (q_rand - q_nearest).norm();
//     double delta_q_unit = (q_rand - q_nearest) / delta_q_mag;
//     Eigen::VectorXd q_new = q_nearest + (delta_q_unit * r);

//     return inLineCollision(q_nearest, q_new, collision_checker); // No intersection found
// }

Eigen::VectorXd GenericRRT::getNewState(const Eigen::VectorXd &q_nearest, const Eigen::VectorXd &q_rand) const
{
    double delta_q_mag = (q_rand - q_nearest).norm();
    Eigen::VectorXd delta_q_unit = (q_rand - q_nearest) / delta_q_mag;
    Eigen::VectorXd q_new = q_nearest + (delta_q_unit * r);
    return q_new;
}

bool GenericRRT::inLineCollision(const Eigen::VectorXd &p1, const Eigen::VectorXd &p2, const amp::ConfigurationSpace &collision_checker) const
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

Eigen::VectorXd GenericRRT::getRandomState()
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

std::pair<amp::Node, Eigen::VectorXd> GenericRRT::getNearestToRand(const Eigen::VectorXd &rand_q, const amp::Node &node_counter)
{
    // std::size_t num_dim = limits.lower_limits.size();
    std::pair<amp::Node, Eigen::VectorXd> q_nearest;
    amp::Node starting_node = 0;
    q_nearest.first = starting_node;
    q_nearest.second = node_to_coord[starting_node];
    double min_distance = (node_to_coord[starting_node] - rand_q).norm();
    double i_distance;
    Eigen::VectorXd q_i;

    for (amp::Node i_node = 0; i_node < node_counter; i_node++)
    {
        q_i = node_to_coord[i_node];
        i_distance = (rand_q - q_i).norm();
        if (i_distance < min_distance)
        {
            q_nearest.first = i_node;
            q_nearest.second = q_i;
            min_distance = i_distance;
        }
    }
    return q_nearest;
}