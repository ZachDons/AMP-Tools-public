#include "MyHW7.h"
#include "MySampleAlgo.h"
#include "AMPCore.h"

amp::Path2D MyPRM2D::plan(const amp::Problem2D &problem)
{
    amp::Path2D path_2d;
    //return path_2d;
    // Make a collsion checker object
    PointAgentCspace cspace(problem);

    // Call the generic planner
    amp::Path path_nd = GenericPRM::plan(problem.q_init, problem.q_goal, cspace);

    // Convert the ND path to a 2D path and return it...
    LOG("2D path converstion");
    int size_path_nd = path_nd.waypoints.size();
    LOG("path size: " << size_path_nd);
    Eigen::VectorXd i_path_nd;
    Eigen::Vector2d i_path_2d;
    for (int i_path = 0; i_path < size_path_nd; i_path++)
    {
        i_path_nd = path_nd.waypoints[i_path];
        i_path_2d[0] = i_path_nd[0];
        i_path_2d[1] = i_path_nd[1];
        path_2d.waypoints.push_back(i_path_2d);
    }
    return path_2d;
}
