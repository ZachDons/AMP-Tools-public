#include "MyHW7.h"
#include "MySampleAlgo.h"
#include "AMPCore.h"

amp::Path2D MyPRM2D::plan(const amp::Problem2D &problem)
{
    // Make a collsion checker object
    PointAgentCspace cspace(problem);

    // Call the generic planner
    amp::Path path_nd = GenericPRM::plan(problem.q_init, problem.q_goal, cspace);

    // Convert the ND path to a 2D path and return it...
    amp::Path2D path_2d;
    int size_path_nd = path_nd.waypoints.size();
    Eigen::VectorXd i_path_nd;
    Eigen::Vector2d i_path_2d;
    for (int i_path = 0; i_path < size_path_nd; i_path++)
    {
        i_path_nd = path_nd.waypoints[i_path];
        i_path_2d = Eigen::Vector2d(i_path_nd[0],i_path_nd[1]);
        path_2d.waypoints.push_back(i_path_2d);

    }
    return path_2d;
}





// bool MyConfigurationSpace::inCollision(const Vector2d &q_i)
// {
//     int num_obs = all_obstacles.size();
//     bool collision_at_state;
//     for (int i_obs = 0; i_obs < num_obs; i_obs++)
//     {
//         collision_at_state = isPointInPolygon(all_obstacles[i_obs], q_i);
//         if (collision_at_state)
//         {
//             operator()(i_x0, i_x1) = 1;
//         }
//     }
// }

