#include "MyBugAlgorithm.h"
#include <cmath>

// Q: why does include iostream not
using namespace std;

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D &problem)
{
    LOG("Starting bug algorithm...");

    // // //////////////////
    // // ///// Bug 2: /////
    // // //////////////////

    // // Set bug inital position
    // path.waypoints.push_back(problem.q_init);
    // bug_position = problem.q_init; // bug start postion
    // Eigen::Vector2d start_init(0, 0.1);
    // bug_position = bug_position+start_init;

    // // Initial heading to goal
    // vec_init_to_goal = (problem.q_goal - problem.q_init);    // vector from q_init to q_goal
    // Eigen::Vector2d unit_vec_init_to_goal = vec_init_to_goal.normalized();     // Compute unit vector from q_init to q_goal
    // double mag_init_to_goal = vec_init_to_goal.norm();     // distance from q_init to q_goal

    // // Create bug parameters
    // bug_step_len = mag_init_to_goal / 500;     // set the bug step distance
    // bug_rot_angle = (std::numbers::pi) / (10); // bug rotation amount

    // // Create bug antenna parameters
    // tenna_mag = bug_step_len + (bug_step_len * 0.15); // antenna size is 5% longer than step size
    // tenna_rot_angle = -(std::numbers::pi) / 2; // 90deg CW rotation of right antenna
    // // Create bug antennas
    // front_tenna_pos = bug_position + (unit_vec_init_to_goal * tenna_mag);                              // antenna directly in front of bug
    // Eigen::Vector2d right_tenna_vector = RotMat((unit_vec_init_to_goal * tenna_mag), tenna_rot_angle); // rotate antenna 90deg to the right
    // right_tenna_pos = bug_position + right_tenna_vector;                                               // antenna to the right of bug

    // // While loop paramters
    // int max_loop_size = 500; // bug start iteration
    // for (int i_loop = 0; i_loop < max_loop_size; i_loop++) // bug_position != problem.q_goal
    // {
    //     // Definition of reaching goal
    //     double di_to_goal = (problem.q_goal - bug_position).norm(); // distance from q_Li to q_goal

    //     // Conditional values
    //     bool goal_reached = (di_to_goal < bug_step_len); // boolean for if the goal is reached
    //     bool obstacle_hit = is_in_obstacles(problem, front_tenna_pos);

    //     // Take a step if the goal has not been reached AND we do not hit an obstacle
    //     if (goal_reached)
    //     {
    //         LOG("Main loop. You reached the goal!");
    //         path.waypoints.push_back(problem.q_goal);
    //         return path;
    //     }
    //     else if (obstacle_hit)
    //     {
    //         LOG("Main loop. Hit obstacle. Enter Circumnavigate.");
    //         //LOG("Bug Pos: " << bug_position << " Front Antenna: " << front_tenna_pos << " Right Antenna: " << right_tenna_pos);
    //         // Circumnavigate the obstacle
    //         CircumnavigateBug2(problem);
    //         ReOrientBug2(problem);

    //         //break;
    //         // Go to Leave Point
    //         // GoToLeavePoint(problem);

    //     }
    //     else // Go toward goal
    //     {
    //         //LOG("Main loop: Take Step");
    //         // Find the direction to the goal
    //         Eigen::Vector2d bug_vec_to_goal = (problem.q_goal - bug_position);
    //         Eigen::Vector2d bug_unitvec_to_goal = bug_vec_to_goal.normalized();
    //         // Take a step in direction of goal (along m-line), update bug and antenna position
    //         TakeStep(bug_unitvec_to_goal);
    //     }
    // }

    // return path;

    // //////////////////
    // ///// Bug 1: /////
    // //////////////////

    // Set bug inital position
    path.waypoints.push_back(problem.q_init);
    bug_position = problem.q_init; // bug start postion
    Eigen::Vector2d start_init(0, 0.1);
    bug_position = bug_position + start_init;

    // Initial heading to goal
    vec_init_to_goal = (problem.q_goal - problem.q_init);                  // vector from q_init to q_goal
    Eigen::Vector2d unit_vec_init_to_goal = vec_init_to_goal.normalized(); // Compute unit vector from q_init to q_goal
    double mag_init_to_goal = vec_init_to_goal.norm();                     // distance from q_init to q_goal

    // Create bug parameters
    bug_step_len = mag_init_to_goal / 500;     // set the bug step distance
    bug_rot_angle = (std::numbers::pi) / (20); // bug rotation amount

    // Create bug antenna parameters
    tenna_mag = bug_step_len + (bug_step_len * 0.15); // antenna size is 5% longer than step size
    tenna_rot_angle = -(std::numbers::pi) / 2;        // 90deg CW rotation of right antenna
    // Create bug antennas
    front_tenna_pos = bug_position + (unit_vec_init_to_goal * tenna_mag);                              // antenna directly in front of bug
    Eigen::Vector2d right_tenna_vector = RotMat((unit_vec_init_to_goal * tenna_mag), tenna_rot_angle); // rotate antenna 90deg to the right
    right_tenna_pos = bug_position + right_tenna_vector;                                               // antenna to the right of bug

    // While loop paramters
    int max_loop_size = 500;                               // bug start iteration
    for (int i_loop = 0; i_loop < max_loop_size; i_loop++) // bug_position != problem.q_goal
    {
        // Definition of reaching goal
        double di_to_goal = (problem.q_goal - bug_position).norm(); // distance from q_Li to q_goal

        // Conditional values
        bool goal_reached = (di_to_goal < bug_step_len); // boolean for if the goal is reached
        bool obstacle_hit = is_in_obstacles(problem, front_tenna_pos);

        // Take a step if the goal has not been reached AND we do not hit an obstacle
        if (goal_reached)
        {
            LOG("Main loop. You reached the goal!");
            path.waypoints.push_back(problem.q_goal);
            return path;
        }
        else if (obstacle_hit)
        {
            LOG("Main loop. Hit obstacle. Enter Circumnavigate.");
            // LOG("Bug Pos: " << bug_position << " Front Antenna: " << front_tenna_pos << " Right Antenna: " << right_tenna_pos);
            //  Circumnavigate the obstacle
            Circumnavigate(problem);
            // Go to Leave Point
            GoToLeavePoint(problem);
        }
        else // Go toward goal
        {
            // LOG("Main loop: Take Step");
            //  Find the direction to the goal
            Eigen::Vector2d bug_vec_to_goal = (problem.q_goal - bug_position);
            Eigen::Vector2d bug_unitvec_to_goal = bug_vec_to_goal.normalized();
            // Take a step in direction of goal (along m-line), update bug and antenna position
            TakeStep(bug_unitvec_to_goal);
        }
    }

    return path;
}

//     // //////////////////
//     // ///// Bug 2: /////
//     // //////////////////
void MyBugAlgorithm::CircumnavigateBug2(const amp::Problem2D &problem)
{
    Eigen::Vector2d obstacle_hit_point = bug_position;
    path_index_obs_hit = path.waypoints.size() - 1;

    // Set "while" loop conditions: exit when arrived back at hit point
    double circumnav_angle_criteria = (std::numbers::pi) / 60; // 4 deg
    double circumnav_step_criteria = 100;
    int circumnav_step_iteration = 1;
    bool circumnav_angle_condition = true;
    bool circumnav_step_contition = true;

    // Loop until arrived back at m-line
    while (circumnav_angle_condition || circumnav_step_contition)
    {
        // Test whether antennas are in obstacle
        bool front_tenna_free = !is_in_obstacles(problem, front_tenna_pos);
        bool right_tenna_free = !is_in_obstacles(problem, right_tenna_pos);
        // if front antenna free && right antenna free
        // step && rotate CW
        if (front_tenna_free && right_tenna_free)
        {
            // Rotate bug back to obstacle
            Rotate(-bug_rot_angle);
            // Step bug
            Eigen::Vector2d unit_vec_front_tenna = (front_tenna_pos - bug_position).normalized();
            TakeStep(unit_vec_front_tenna);
            // Increase step counter
            circumnav_step_iteration = circumnav_step_iteration + 1;
        }
        // if front antenna free && right antenna not free
        // step
        else if (front_tenna_free && !right_tenna_free)
        {
            // Step bug
            Eigen::Vector2d unit_vec_front_tenna = (front_tenna_pos - bug_position).normalized();
            TakeStep(unit_vec_front_tenna);
            // Increase step counter
            circumnav_step_iteration = circumnav_step_iteration + 1;
        }
        // if front antenna not free && right antenna free
        // rotate
        else if (!front_tenna_free && right_tenna_free)
        {
            // Rotate bug
            Rotate(bug_rot_angle);
        }
        // if front antenna not free && right antenna not free
        // rotate
        else if (!front_tenna_free && !right_tenna_free)
        {
            // Rotate bug
            Rotate(bug_rot_angle);
        }
        else
        {
            LOG("ZACH ERROR: You hit the ELSE STATEMENT in the Maneuver Phase");
        }

        // Update while loop conditional criteria

        // Angle between bug->goal and init->goal
        Eigen::Vector2d vec_bug_to_goal = problem.q_goal - bug_position;
        // Calculate the dot product and magnitudes of the vectors
        double dotProduct = vec_init_to_goal.dot(vec_bug_to_goal);
        double magnitude1 = vec_init_to_goal.norm();
        double magnitude2 = vec_bug_to_goal.norm();
        // Calculate the cosine of the angle between the vectors using the dot product
        double cosAngle = dotProduct / (magnitude1 * magnitude2);
        // Calculate the angle in radians
        double angleRad = (std::acos(cosAngle));
        // LOG("Angle between vecs: "<< angleRad);
        //  if (circumnav_step_iteration > 500)
        //  {
        //      LOG("long while loop run time...breaking");
        //      break;
        //  }

        // Exit Criteria
        //  Bug is close to m-line
        circumnav_angle_condition = (angleRad > circumnav_angle_criteria);
        // Bug has stepped enough to get out of the initial circumnav distance: circumnav_step_criteria < circumnav_step_iteration
        circumnav_step_contition = (circumnav_step_iteration < circumnav_step_criteria);
    }
}

void MyBugAlgorithm::ReOrientBug2(const amp::Problem2D &problem)
{
    // Find the direction to the goal
    Eigen::Vector2d bug_vec_to_goal = (problem.q_goal - bug_position);
    Eigen::Vector2d bug_unitvec_to_goal = bug_vec_to_goal.normalized();
    // Rotate antenna to goals
    front_tenna_pos = bug_position + (bug_unitvec_to_goal * tenna_mag);
    Eigen::Vector2d right_tenna_vector = RotMat((bug_unitvec_to_goal * tenna_mag), tenna_rot_angle); // rotate antenna 90deg to the right
    right_tenna_pos = bug_position + right_tenna_vector;
}

//     // //////////////////
//     // ///// Bug 1: /////
//     // //////////////////

// amp::Path2D MyBugAlgorithm::Bug1(const amp::Problem2D &problem, const amp::Path2D &path)
// {
//     // //////////////////
//     // ///// Bug 1: /////
//     // //////////////////

//     // Set bug inital position
//     path.waypoints.push_back(problem.q_init);
//     bug_position = problem.q_init; // bug start postion
//     Eigen::Vector2d start_init(0, 0.1);
//     bug_position = bug_position+start_init;

//     // Initial heading to goal
//     Eigen::Vector2d vec_init_to_goal = (problem.q_goal - problem.q_init);    // vector from q_init to q_goal
//     Eigen::Vector2d unit_vec_init_to_goal = vec_init_to_goal.normalized();     // Compute unit vector from q_init to q_goal
//     double mag_init_to_goal = vec_init_to_goal.norm();     // distance from q_init to q_goal

//     // Create bug parameters
//     bug_step_len = mag_init_to_goal / 500;     // set the bug step distance
//     bug_rot_angle = (std::numbers::pi) / (20); // bug rotation amount

//     // Create bug antenna parameters
//     tenna_mag = bug_step_len + (bug_step_len * 0.15); // antenna size is 5% longer than step size
//     tenna_rot_angle = -(std::numbers::pi) / 2; // 90deg CW rotation of right antenna
//     // Create bug antennas
//     front_tenna_pos = bug_position + (unit_vec_init_to_goal * tenna_mag);                              // antenna directly in front of bug
//     Eigen::Vector2d right_tenna_vector = RotMat((unit_vec_init_to_goal * tenna_mag), tenna_rot_angle); // rotate antenna 90deg to the right
//     right_tenna_pos = bug_position + right_tenna_vector;                                               // antenna to the right of bug

//     // While loop paramters
//     int max_loop_size = 500; // bug start iteration
//     for (int i_loop = 0; i_loop < max_loop_size; i_loop++) // bug_position != problem.q_goal
//     {
//         // Definition of reaching goal
//         double di_to_goal = (problem.q_goal - bug_position).norm(); // distance from q_Li to q_goal

//         // Conditional values
//         bool goal_reached = (di_to_goal < bug_step_len); // boolean for if the goal is reached
//         bool obstacle_hit = is_in_obstacles(problem, front_tenna_pos);

//         // Take a step if the goal has not been reached AND we do not hit an obstacle
//         if (goal_reached)
//         {
//             LOG("Main loop. You reached the goal!");
//             path.waypoints.push_back(problem.q_goal);
//             return path;
//         }
//         else if (obstacle_hit)
//         {
//             LOG("Main loop. Hit obstacle. Enter Circumnavigate.");
//             //LOG("Bug Pos: " << bug_position << " Front Antenna: " << front_tenna_pos << " Right Antenna: " << right_tenna_pos);
//             // Circumnavigate the obstacle
//             Circumnavigate(problem);
//             // Go to Leave Point
//             GoToLeavePoint(problem);
//         }
//         else // Go toward goal
//         {
//             //LOG("Main loop: Take Step");
//             // Find the direction to the goal
//             Eigen::Vector2d bug_vec_to_goal = (problem.q_goal - bug_position);
//             Eigen::Vector2d bug_unitvec_to_goal = bug_vec_to_goal.normalized();
//             // Take a step in direction of goal (along m-line), update bug and antenna position
//             TakeStep(bug_unitvec_to_goal);
//         }
//     }
//     return path;
// }

void MyBugAlgorithm::GoToLeavePoint(const amp::Problem2D &problem)
{
    // Setup
    int num_all_waypoints = path.waypoints.size(); // Number of all previous waypoints
    // Initalize loop variables
    double test_dist_waypt = (problem.q_goal - problem.q_init).norm(); // distance from q_init to q_goal
    double bugobs_min_dist = (problem.q_goal - problem.q_init).norm(); // distance from q_init to q_goal
    int bugobs_min_dist_index = 0;

    // Find the closest waypoint and index of that waypoint to goal
    for (int i_waypt = 0; i_waypt < num_all_waypoints; i_waypt++) // bug_position != problem.q_goal
    {
        test_dist_waypt = (problem.q_goal - path.waypoints[i_waypt]).norm();
        if (test_dist_waypt < bugobs_min_dist)
        {
            bugobs_min_dist = test_dist_waypt;
            bugobs_min_dist_index = i_waypt;
        }
    }

    // Index difference between when the bug hit the obstacle and when it circumnavigated
    int diff_obs_hit_leave = bugobs_min_dist_index - path_index_obs_hit;
    // Append these points to waypoints
    for (int i_to_leavept = 0; i_to_leavept < diff_obs_hit_leave; i_to_leavept++) // bug_position != problem.q_goal
    {
        int add_obspath_index = path_index_obs_hit + i_to_leavept;
        path.waypoints.push_back(path.waypoints[add_obspath_index]);
    }
    // Update bug position to leave point (closest point to goal)
    bug_position = path.waypoints[path_index_obs_hit + diff_obs_hit_leave];

    // Find the direction to the goal
    Eigen::Vector2d bug_vec_to_goal = (problem.q_goal - bug_position);
    Eigen::Vector2d bug_unitvec_to_goal = bug_vec_to_goal.normalized();
    // Rotate antenna to goals
    front_tenna_pos = bug_position + (bug_unitvec_to_goal * tenna_mag);
    Eigen::Vector2d right_tenna_vector = RotMat((bug_unitvec_to_goal * tenna_mag), tenna_rot_angle); // rotate antenna 90deg to the right
    right_tenna_pos = bug_position + right_tenna_vector;
}

void MyBugAlgorithm::Circumnavigate(const amp::Problem2D &problem)
{
    Eigen::Vector2d obstacle_hit_point = bug_position;
    path_index_obs_hit = path.waypoints.size() - 1;

    // Set "while" loop conditions: exit when arrived back at hit point
    double circumnav_dist_criteria = bug_step_len * 3;
    double circumnav_step_criteria = 100;
    int circumnav_step_iteration = 1;
    bool circumnav_dist_condition = true;
    bool circumnav_step_contition = true;

    // Loop until arrived back at hit point
    while (circumnav_dist_condition || circumnav_step_contition)
    {
        // Test whether antennas are in obstacle
        bool front_tenna_free = !is_in_obstacles(problem, front_tenna_pos);
        bool right_tenna_free = !is_in_obstacles(problem, right_tenna_pos);
        // if front antenna free && right antenna free
        // step && rotate CW
        if (front_tenna_free && right_tenna_free)
        {
            // Rotate bug back to obstacle
            Rotate(-bug_rot_angle);
            // Step bug
            Eigen::Vector2d unit_vec_front_tenna = (front_tenna_pos - bug_position).normalized();
            TakeStep(unit_vec_front_tenna);
            // Increase step counter
            circumnav_step_iteration = circumnav_step_iteration + 1;
        }
        // if front antenna free && right antenna not free
        // step
        else if (front_tenna_free && !right_tenna_free)
        {
            // Step bug
            Eigen::Vector2d unit_vec_front_tenna = (front_tenna_pos - bug_position).normalized();
            TakeStep(unit_vec_front_tenna);
            // Increase step counter
            circumnav_step_iteration = circumnav_step_iteration + 1;
        }
        // if front antenna not free && right antenna free
        // rotate
        else if (!front_tenna_free && right_tenna_free)
        {
            // Rotate bug
            Rotate(bug_rot_angle);
        }
        // if front antenna not free && right antenna not free
        // rotate
        else if (!front_tenna_free && !right_tenna_free)
        {
            // Rotate bug
            Rotate(bug_rot_angle);
        }
        else
        {
            LOG("ZACH ERROR: You hit the ELSE STATEMENT in the Maneuver Phase");
        }

        // Update while loop conditional criteria

        // Distance from hit point to bugs position aroud obstacle
        double circumnav_dist = (obstacle_hit_point - bug_position).norm();

        // Exit Criteria
        //  Bug is close to original hit point: circumnav_dist < circumnav_dist_criteria
        circumnav_dist_condition = (circumnav_dist > circumnav_dist_criteria);
        // Bug has stepped enough to get out of the initial circumnav distance: circumnav_step_criteria < circumnav_step_iteration
        circumnav_step_contition = (circumnav_step_iteration < circumnav_step_criteria);
    }
}

void MyBugAlgorithm::Rotate(double theta)
{
    // rotate a small amout
    // Front antenna
    Eigen::Vector2d front_tenna_vec = front_tenna_pos - bug_position;     // subtract bug position from antenna position to get antenna vector
    Eigen::Vector2d front_tenna_rot_vec = RotMat(front_tenna_vec, theta); // rotate antenna vector
    front_tenna_pos = bug_position + front_tenna_rot_vec;                 // add rotated antenna vector to bug position to get antenna position
    // Right antenna
    Eigen::Vector2d right_tenna_vec = right_tenna_pos - bug_position;     // subtract bug position from antenna position to get antenna vector
    Eigen::Vector2d right_tenna_rot_vec = RotMat(right_tenna_vec, theta); // rotate antenna vector
    right_tenna_pos = bug_position + right_tenna_rot_vec;                 // add rotated antenna vector to bug position to get antenna position
}

void MyBugAlgorithm::TakeStep(Eigen::Vector2d unit_vec)
{
    bug_position = bug_position + (bug_step_len * unit_vec);
    front_tenna_pos = front_tenna_pos + (bug_step_len * unit_vec);
    right_tenna_pos = right_tenna_pos + (bug_step_len * unit_vec);
    path.waypoints.push_back(bug_position);
}

bool MyBugAlgorithm::is_in_obstacles(const amp::Problem2D &problem, Eigen::Vector2d bug_position)
{
    for (int i_obs = 0; i_obs < problem.obstacles.size(); i_obs++)
    {
        // Assign individual obstacle from set of obstacles
        amp::Polygon obs = problem.obstacles[i_obs];
        // Number of vertices in an individual obstacle
        int num_obs_vertices = obs.verticesCCW().size();

        // Create array of primitive values for each obstacle
        Eigen::ArrayXd obs_primitive_array(num_obs_vertices);
        // cout << "preallocate prim array: " << obs_primitive_array << endl;

        // check if bug is contained in the union of all primatives for a given obstacle
        for (int j_vertex = 0; j_vertex < num_obs_vertices; j_vertex++)
        {
            // Allocate first vertex
            Eigen::Vector2d vertex_a = obs.verticesCCW()[j_vertex];
            // cout << "vertex " << j_vertex << ": " <<vertex_1 << endl;
            //   Allocate second vertex. Wrap back to the obstacles first vertex if needed
            //   Q: how can I intelligently initialize a variable?
            Eigen::Vector2d vertex_b = obs.verticesCCW()[j_vertex];
            if (j_vertex < (num_obs_vertices - 1))
            {

                vertex_b = obs.verticesCCW()[j_vertex + 1];
                // cout << "j+1 vertex = " << vertex_b << endl;
            }
            else // Wrap back to the obstacles first vertex
            {
                // cout << "j0 vertex" << endl;
                vertex_b = obs.verticesCCW()[0];
            }
            // cout << "vert B check  = " << vertex_b << endl;

            obs_primitive_array[j_vertex] = is_in_primative(vertex_a, vertex_b, bug_position);
            // cout << "print adjusted arra: " << obs_primitive_array << endl;
        }

        // LOG("Here is the primitive array: " << obs_primitive_array);
        //  If the bug is in the obstacle
        if (all_elements_negative(obs_primitive_array, num_obs_vertices))
        {
            bool obstacle_check = true;

            // LOG("Inside Obstacle Num = " << i_obs << " | Detector Position = " << bug_position );
            return obstacle_check;
        }
    }

    bool obstacle_check = false;
    return obstacle_check;
}

double MyBugAlgorithm::is_in_primative(Eigen::Vector2d vertex_1, Eigen::Vector2d vertex_2, Eigen::Vector2d bug_position)
{
    double bug_position_x = bug_position[0];
    double bug_position_y = bug_position[1];
    double vertex_1_x = vertex_1[0];
    double vertex_1_y = vertex_1[1];
    double vertex_2_x = vertex_2[0];
    double vertex_2_y = vertex_2[1];

    double primative = ((vertex_1_y - vertex_2_y) * bug_position_x) + ((vertex_2_x - vertex_1_x) * bug_position_y) + ((vertex_1_x * vertex_2_y) - (vertex_2_x * vertex_1_y));
    double bug_in_primative = (primative <= 0);
    // if (bug_in_primative)
    // {
    //     //cout << "Inside primitive: Obstacle Num = " << i_obs << " | first vertex = " << vertex_a << endl;
    //     cout << "Primitive tripwire: " << primative << endl;
    //     cout << "Vertex 1: " << vertex_1 << " Vertex 2: " << vertex_2 << " bug: " << bug_position<< endl;
    // }
    return bug_in_primative;
}

// Returns
bool MyBugAlgorithm::all_elements_negative(Eigen::ArrayXd element_array, int num_elements)
{
    for (int i_element = 0; i_element < num_elements; i_element++)
    {
        if (element_array[i_element] > 0.0) // If any number is non-negative, return false
        {
            return false;
        }
    }
    return true; // All numbers are negative, return true
}

Eigen::Vector2d MyBugAlgorithm::RotMat(Eigen::Vector2d r_before, double theta_rot)
{
    // Rotation matrix
    Eigen::MatrixXd rot_mat(2, 2);
    rot_mat(0, 0) = cos(theta_rot);
    rot_mat(0, 1) = -sin(theta_rot);
    rot_mat(1, 0) = sin(theta_rot);
    rot_mat(1, 1) = cos(theta_rot);
    // Perform rotation
    Eigen::Vector2d r_after = rot_mat * r_before;
    return r_after;
}

/*
bool MyBugAlgorithm::do_lines_intersect(const amp::Problem2D &problem, Eigen::Vector2d prior_bug_position, Eigen::Vector2d bug_position)
{
    for (int i_obs = 0; i_obs < problem.obstacles.size(); i_obs++)
    {
        // Assign individual obstacle from set of obstacles
        amp::Polygon obs = problem.obstacles[i_obs];
        // Number of vertices in an individual obstacle
        int num_obs_vertices = obs.verticesCCW().size();

        // check if bug is contained in the union of all primatives for a given obstacle
        for (int j_vertex = 0; j_vertex < num_obs_vertices; j_vertex++)
        {
            // Allocate first vertex
            Eigen::Vector2d vertex_a = obs.verticesCCW()[j_vertex];

            // Allocate second vertex. Wrap back to the obstacles first vertex if needed
            Eigen::Vector2d vertex_b = obs.verticesCCW()[j_vertex];

            // Assign the vertices
            if (j_vertex < (num_obs_vertices - 1))
            {
                vertex_b = obs.verticesCCW()[j_vertex + 1];
                // cout << "j+1 vertex = " << vertex_b << endl;
            }
            else // Wrap back to the obstacles first vertex
            {
                // cout << "j0 vertex" << endl;
                vertex_b = obs.verticesCCW()[0];
            }

            //Eigen::Hyperplane<double,2>::through(p1,p2)
            //plane1.intersection(plane)

            // Compartmentalize bug line
            double x3 = prior_bug_position[0];
            double y3 = prior_bug_position[1];
            double x4 = bug_position[0];
            double x4 = bug_position[1];

            // Compartmentalize vertex line
            double x1 = vertex_a[0];
            double y1 = vertex_a[1];
            double x2 = vertex_b[0];
            double x2 = vertex_b[1];

            // Intersection points of the two infinite lines
            Px = ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3x4) ) / ( ((x1-x2)*(y3-y4)) - ((y1-y2)*(x3-x4)) );

        }

        if (all_elements_negative(obs_primitive_array,num_obs_vertices))
        {
            bool obstacle_check = true;
            //cout << "Inside primitive: Obstacle Num = " << i_obs << " | first vertex = " << vertex_a << endl;
            return obstacle_check;
        }
    }

    bool obstacle_check = false;
    return obstacle_check;
}*/
