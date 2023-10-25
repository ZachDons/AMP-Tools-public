#include "MyGradientDescent.h"
#include "AMPCore.h"

// using namespace amp;

// MyGradientDescent::MyGradientDescent(double d_star, double q_star, double epsilon, double eta, double step_size) :
//     d_star(d_star),
//     q_star(q_star),
//     epsilon(epsilon),
//     eta(eta),
//     step_size(step_size)
// {
//     // Constructor implementation here if needed
// }

// Transformation matrix

amp::Path2D MyGradientDescent::plan(const amp::Problem2D &problem)
{
    // initialize path object
    amp::Path2D path;

    // initialize starting variables
    Vector2d current_position = problem.q_init;

    path.waypoints.push_back(current_position);

    // loop through the gradient potential
    int max_cnt = 500;
    int iter_cnt = 0;
    calc_gradient_potential(problem, current_position);
    cout << "q_goal: " << problem.q_goal << std::endl;
    cout << "Gradient Potential: " << gradient_potential << std::endl;
    while (iter_cnt < max_cnt) // while the gradient potential is not zero
    {
        // take a step in direction of potential
        current_position = current_position - step_size * gradient_potential;
        // Calculate new gradient potential
        calc_gradient_potential(problem, current_position);
        // update path
        path.waypoints.push_back(current_position);

        cout << "Posten Coords: " << current_position << std::endl;
        // if (iter_cnt > 5000)
        // {
        //     break;
        // }
        iter_cnt = iter_cnt + 1;
    }
    path.waypoints.push_back(problem.q_goal);
    double path_len = path.length();
    cout << "PATH LENGTH: " << path_len << std::endl;
    return path;
}

bool MyGradientDescent::zero_gradient_potential()
{
    // Preallocate: gradient potential is not "zero"
    bool grad_pot_is_zero = false;
    // Determine if gradient potential is "zero"
    double mag_gradient_potential = gradient_potential.norm(); // magnitude of gradient
    cout << "Mag gradient potential: " << mag_gradient_potential << std::endl;

    if (mag_gradient_potential <= epsilon)
    {
        grad_pot_is_zero = true; // gradient potential is "zero"
    }
    cout << "The gradient potential is zero: " << grad_pot_is_zero << std::endl;
    return grad_pot_is_zero;
}

double MyGradientDescent::find_distance(const Vector2d point_1, const Vector2d point_2)
{
    double point_1_x = point_1.x();
    double point_1_y = point_1.y();
    double point_2_x = point_2.x();
    double point_2_y = point_2.y();

    double dx = point_1_x - point_2_x;
    double dy = point_1_y - point_2_y;
    return std::sqrt(dx * dx + dy * dy);
}

void MyGradientDescent::calc_gradient_potential(const amp::Problem2D &problem, const Vector2d q_pos)
{
    // // Define the attractive gradient potential
    Vector2d attractive_grad_pot = calc_attractive_grad_pot(problem, q_pos);
    cout << "Attract Grav Pot: " << attractive_grad_pot << std::endl;

    // Define the repulsive gradient potential
    Vector2d repulsive_grad_pot = calc_repulsive_grad_pot(problem, q_pos);

    // Noise
    Vector2d noise_grad_pot = generate_noise();

    // Cumulative gradient potential

    gradient_potential = attractive_grad_pot + repulsive_grad_pot + noise_grad_pot;
    cout << "Rep Pot: " << repulsive_grad_pot << std::endl;
    cout << "Att Pot: " << attractive_grad_pot << std::endl;
    cout << "Total Grav Pot: " << gradient_potential << std::endl;
}

Vector2d MyGradientDescent::calc_attractive_grad_pot(const amp::Problem2D &problem, const Vector2d q_pos)
{
    // Define the attractive gradient potential
    Vector2d attractive_grad_pot;
    double dist_q_to_q_goal = find_distance(q_pos, problem.q_goal); // distance from q to q_goal
    if (dist_q_to_q_goal <= d_star)
    {
        attractive_grad_pot = zeta * (q_pos - problem.q_goal);
    }
    else
    {
        attractive_grad_pot = (d_star * zeta * (q_pos - problem.q_goal)) / dist_q_to_q_goal;
    }
    return attractive_grad_pot;
}

Vector2d MyGradientDescent::calc_repulsive_grad_pot(const amp::Problem2D &problem, const Vector2d q_pos)
{
    // Preallocate
    int num_obstacles = problem.obstacles.size();
    Vector2d single_obs_rep_GP;
    Vector2d repulsive_grad_pot;

    // Loop through all obstacles
    for (int i_obs = 0; i_obs < num_obstacles; i_obs++)
    {
        // Assign individual obstacle from set of obstacles
        amp::Polygon single_obs = problem.obstacles[i_obs];
        Vector2d qMc_per_obs = qMc_to_obs(single_obs.verticesCCW(), q_pos);
        double mag_qMc = qMc_per_obs.norm();
        if (mag_qMc <= q_star)
        {
            single_obs_rep_GP = eta * ((1 / q_star) - (1 / mag_qMc)) * (qMc_per_obs / (mag_qMc * mag_qMc * mag_qMc));
        }
        else
        {
            single_obs_rep_GP = Vector2d::Zero();
        }
        repulsive_grad_pot = repulsive_grad_pot + single_obs_rep_GP;
        cout << "Obs " << i_obs << ": dmin = " << mag_qMc << std::endl;
    }

    // Define the attractive gradient potential
    // repulsive_grad_pot = {0.0, 0.0};
    return repulsive_grad_pot;
}

Vector2d MyGradientDescent::qMc_to_obs(const vector<Vector2d> obs_vertices, const Vector2d q_pos)
{
    double dist_to_vertex;
    double min_distance_limit = std::numeric_limits<double>::max();
    Vector2d closest_point;
    Vector2d c_to_q;

    int num_obs_vertices = obs_vertices.size();

    // Loop through all vertices
    for (int j_vertex = 0; j_vertex < num_obs_vertices; j_vertex++)
    {
        // Allocate first vertex
        Vector2d vertex_a = obs_vertices[j_vertex];
        // Allocate seoond vertex.
        Vector2d vertex_b;
        if (j_vertex < (num_obs_vertices - 1))
        {
            vertex_b = obs_vertices[j_vertex + 1];
        }
        else // Wrap back to the obstacles first vertex
        {
            vertex_b = obs_vertices[0];
        }

        // Compute the dot product
        double rdot = dot_product(vertex_a, vertex_b, q_pos);

        // cout << "Vertex A: " << vertex_a << std::endl;
        // cout << "Vertex B: " << vertex_b << std::endl;
        // cout << "Q-pos: " << q_pos << std::endl;
        // cout << "Dot Prod: " << rdot << std::endl;

        if ((0 < rdot) && (rdot < 1))
        {

            // double dist_ab = find_distance(vertex_a, vertex_b);
            // double dist_a = rdot * dist_ab;
            // double dist_aq = find_distance(vertex_a, q_pos);
            Vector2d vec_closestpoint_a = (rdot * (vertex_b - vertex_a)) + vertex_a;
            // Vector2d vec_closestpoint_a = (1.0-rdot)*vertex_a + (rdot*vertex_b);
            // Vector2d vec_a_q = q_pos - vertex_a;
            c_to_q = q_pos - vec_closestpoint_a;
            PRINT_VEC2("Closest point on edge: ", vec_closestpoint_a);
            cout << "D-min computed for edge: " << j_vertex << "| Dmin = " << c_to_q.norm() << std::endl;
            // double dist_min_to_edge = std::sqrt((dist_aq * dist_aq) - (dist_a * dist_a));
            return c_to_q;
        }



        dist_to_vertex = (find_distance(vertex_a, q_pos));
        if (dist_to_vertex < min_distance_limit)
        {
            min_distance_limit = dist_to_vertex;
            closest_point = vertex_a;
        }
    }
    // for (int ii = 0; ii < dist_to_vertex.size(); ii++){
    //     cout << "Distances computed to vertices: " << dist_to_vertex[ii] << std::endl;
    // }
    // cout << "Distances computed to vertices: " << dist_to_vertex << std::endl;
    // Find the minimum value in the vector
    // d_min = *std::min_element(dist_to_vertex.begin(), dist_to_vertex.end());

    c_to_q = q_pos - closest_point;
    return c_to_q;
}

Vector2d MyGradientDescent::generate_noise()
{
    //std::srand(static_cast<unsigned int>(std::time(nullptr)));
    Vector2d noise_grad = 0.2 * Vector2d::Random();
    cout << "Total noise Pot: " << noise_grad << std::endl;

    return noise_grad;
}

double MyGradientDescent::dot_product(const Vector2d vertex_a, const Vector2d vertex_b, const Vector2d q_pos)
{
    // Compute vectors
    Vector2d edge_ab = vertex_b - vertex_a;
    Vector2d vector_aq = q_pos - vertex_a;
    // Calculate the dot product
    double dotProduct = edge_ab.dot(vector_aq);

    // Calculate the dot product using std::inner_product
    return dotProduct;
}