#include "HelpfulClass.h"
#include <iostream>
#include <algorithm> // For std::sort
#include <cmath>
using std::vector, Eigen::Vector2d, std::cout;

// void MyClass::hereIsAMethod()
// {
//     // Implementation
// }

// Function to rotate a vector<Vector2d> by an angle in radians
vector<Vector2d> MyClass::Rotate_Vertices(vector<Vector2d> vertex_list, double angle_rad)
{
    // Preallocate list of rotated vertices
    vector<Vector2d> rot_vertex_list;
    // Rotate each vector in the vertex list
    for (const Vector2d& vec : vertex_list) {
        Vector2d rot_vec = Rotate_Vector(vec, angle_rad);
        rot_vertex_list.push_back(rot_vec);     
    }
    return rot_vertex_list;
}

// Function to rotate a Vector2d by an angle in radians
Vector2d MyClass::Rotate_Vector(const Vector2d& vec, double angle_rad) {
    // Define sin and cos
    double cos_theta = std::cos(angle_rad);
    double sin_theta = std::sin(angle_rad);
    // Define the 2D rotation matrix
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos_theta, -sin_theta,
                     sin_theta, cos_theta;
    // Apply the rotation matrix to the vector
    return rotation_matrix * vec;
}

// Sort the vector<Vector2d> in CCW order where the first Vector2d is the smallest y coordinate
vector<Vector2d> MyClass::Sort_Vertices(vector<Vector2d> vertex_list)
{
    // Preallocate vertex lists
    vector<Vector2d> temp_vertex_list = vertex_list;
    vector<Vector2d> new_vertex_list;
    // Sort the vector: smallest y-coord. If two smallest y-coord, pick one with smallest x-coord
    std::stable_sort(temp_vertex_list.begin(), temp_vertex_list.end(), [](const Vector2d& a, const Vector2d& b) {
        if (a.y() == b.y()) {
            return a.x() < b.x(); // Sort by x-component if y-components are equal
        }
        return a.y() < b.y(); // Sort by y-component
    });
    // Find the smallest (first) vector
    Vector2d first_vertex = temp_vertex_list[0];
    // Place the smallest y-component element as the first vector
    int idx_start = 0;
    for (const Eigen::Vector2d& vec : vertex_list) {
        if (vec == first_vertex) {
            new_vertex_list.push_back(vec);
            idx_start = idx_start + 1;
            break;
        }
        idx_start = idx_start + 1;
    }
    // add elements after smallest y-component element
    int num_vertex = vertex_list.size();
    for (int i_vertex = idx_start; i_vertex < num_vertex; i_vertex++)
    {
        new_vertex_list.push_back(vertex_list[i_vertex]);
    }
    // add elements prior smallest y-component element
    int idx_0 = 0;
    for (int i_vertex = idx_0; i_vertex < idx_start-1; i_vertex++)
    {
        new_vertex_list.push_back(vertex_list[i_vertex]);
    }
    return new_vertex_list;
}

// Compute the negative of each element in vector<Vector2d> 
vector<Vector2d> MyClass::Negative_Vertices(vector<Vector2d> vertex_list)
{
    // Take negative of all vertices
    // Multiply each element by -1
    vector<Vector2d> neg_vertex_list;
    // Iterate through the input vector and negate each element
    for (const Vector2d& i_vertex : vertex_list) {
        neg_vertex_list.push_back(-i_vertex); // Negate the vector and add to the result
    }
    return neg_vertex_list;
}

// Compute the angle of a vector given by two points
double MyClass::Vector_Angle(Vector2d point_1, Vector2d point_2)
{
    // Vector between vertices (edge)
    Vector2d edge = point_2 - point_1;
    // Edge components
    double edge_dx = edge.x();
    double edge_dy = edge.y();
    // Calculate the angle in radians using atan2: range(-pi,pi)
    double angle = std::atan2(edge_dy, edge_dx);
    if (angle < 0)
    {
        angle = angle + 2*M_PI;
    }
    return angle;
}

vector<amp::Polygon> MyClass::Create_Cobs(const amp::Obstacle2D &obstacle, vector<Vector2d> robot_vertices, vector<double> angle_rad)
{
    // Obstacle
    // simplify variables
    vector<Vector2d> obstacle_vertices = obstacle.verticesCCW();
    // sort obstacle
    vector<Vector2d> sorted_obstacle_vertices = Sort_Vertices(obstacle_vertices);
    // for (const Eigen::Vector2d& vec : sorted_obstacle_vertices) {
    //     std::cout << vec.transpose() << std::endl;
    // }
    // std::cout << std::endl;

    // Robot
    // Preallocate for loop
    vector<amp::Polygon> Cspace_polygons;
    // loop throup all angles
    int num_angles = angle_rad.size();
    for (int i_angle = 0; i_angle < num_angles; i_angle++)
    {
        // negaive of robot_vertices (permits minkoski difference)
        vector<Vector2d> neg_robot_vertices = Negative_Vertices(robot_vertices);
        // Print the negated vertices
        // for (const Eigen::Vector2d& vec : neg_robot_vertices) {
        //     std::cout << vec.transpose() << std::endl;
        // }
        // std::cout << std::endl;
        
        // rotate
        vector<Vector2d> rotated_robot_vertices = Rotate_Vertices(neg_robot_vertices, angle_rad[i_angle]);
        // for (const Eigen::Vector2d& vec : rotated_robot_vertices) {
        //     std::cout << vec.transpose() << std::endl;
        // }
        // std::cout << std::endl;

        // sort robot
        vector<Vector2d> sorted_robot_vertices = Sort_Vertices(rotated_robot_vertices);
        // for (const Eigen::Vector2d& vec : sorted_robot_vertices) {
        //     std::cout << vec.transpose() << std::endl;
        // }
        // std::cout << std::endl;
        
        //Minkoski Sum
        
        vector<Vector2d> Cobs = Minkoski_Sum(obstacle_vertices, sorted_robot_vertices);
        std::cout << "Cspace-Obs" << std::endl;
        for (const Eigen::Vector2d& vec : Cobs) {
            std::cout << vec.transpose() << std::endl;
        }
        std::cout << std::endl;
        amp::Polygon C_polygon(Cobs);
        Cspace_polygons.push_back(C_polygon);
    }
    
    return Cspace_polygons;    
}

// Compute the Minkoski_Sum
vector<Vector2d> MyClass::Minkoski_Sum(vector<Vector2d> obstacle_vertices, vector<Vector2d> robot_vertices)
{
    // Initialize loop variables
    // Add the first element to the obstacles and robot
    obstacle_vertices.push_back(obstacle_vertices[0]);
    robot_vertices.push_back(robot_vertices[0]);
    // iteration size
    int n_obs = obstacle_vertices.size();
    int m_rob = robot_vertices.size();
    // C-space obstacle vertices
    vector<Vector2d> Cspace_obstacle;
    // counters
    int i_obs = 0;
    int j_rob = 0;

    // Loop
    while ((i_obs < n_obs) && (j_rob < m_rob))
    {
        cout << "Indexs: i = " << i_obs << "j = " << j_rob << std::endl;
        // Save vertices
        Vector2d minkoski_sum = obstacle_vertices[i_obs] + robot_vertices[j_rob];
        Cspace_obstacle.push_back(minkoski_sum);
        cout << std::endl;
        cout << "C-Space obstacle vertices:" << minkoski_sum << std::endl << std::endl;

        // Compute the angle of each edge
        double obs_edge_angle = Vector_Angle(obstacle_vertices[i_obs], obstacle_vertices[i_obs + 1]);
        double rob_edge_angle = Vector_Angle(robot_vertices[j_rob], robot_vertices[j_rob + 1]);
        //cout << "Obstacle: pt_i " << obstacle_vertices[i_obs] << " to pt_i+1 " << obstacle_vertices[i_obs + 1] << std::endl;
        cout << "Obs edge angle = " << obs_edge_angle << std::endl;
        //cout << "Robot: pt_j " << robot_vertices[j_rob] << " to pt_j+1 " << robot_vertices[j_rob + 1] << std::endl;
        cout << "Rob edge angle = " << rob_edge_angle << std::endl;

        if (obs_edge_angle < rob_edge_angle)
        {
            i_obs = i_obs + 1;
        }
        else if (obs_edge_angle > rob_edge_angle)
        {
            j_rob = j_rob + 1;
        }
        else
        {
            i_obs = i_obs + 1;
            j_rob = j_rob + 1;
        }
    }

    return Cspace_obstacle;
}