#pragma once // includes are included only once
#include "AMPCore.h"
#include "hw/HW5.h"
#include <cmath>
#include <iostream>
using std::cout, std::vector, Eigen::Vector2d;

// using namespace amp;
class MyGradientDescent : public amp::GDAlgorithm
{
public:
    // Methods
    // GradientDescent() = default;                   // default constructor
    // GradientDescent(double d_star, double b_star); // constructor -> also need to implement in cpp file
    virtual amp::Path2D plan(const amp::Problem2D &problem) override;
    double find_distance(const Vector2d point_1, const Vector2d point_2);
    bool zero_gradient_potential();
    void calc_gradient_potential(const amp::Problem2D &problem, const Vector2d q_pos);
    Vector2d calc_attractive_grad_pot(const amp::Problem2D &problem, const Vector2d q_pos);
    Vector2d calc_repulsive_grad_pot(const amp::Problem2D &problem, const Vector2d q_pos);
    Vector2d qMc_to_obs(const vector<Vector2d> obs_vertices, const Vector2d q_pos);
    Vector2d generate_noise();

    double dot_product(const Vector2d vertex_a, const Vector2d vertex_b, const Vector2d q_pos);
        MyGradientDescent(double d_star, double q_star, double epsilon, double eta, double zeta, double step_size) : d_star(d_star),
                                                                                                                     q_star(q_star),
                                                                                                                     epsilon(epsilon),
                                                                                                                     eta(eta),
                                                                                                                     zeta(zeta),
                                                                                                                     step_size(step_size) {}

private:
    // Member vars
    double d_star;  // distance to goal
    double q_star;  // distance to obstacle
    double epsilon; // termination distance condition
    double eta;     // gain on repulsize gradient
    double zeta;    // gain on attractive gradient
    double step_size;

    Vector2d gradient_potential;

    amp::Problem2D problem;

    // hw5.h
    // run GD alg on getWorkspace1 and use the HW2 header file
    // check -> same as hw 2. these evaluate your algo against the workspace
    // generate and check -> generates a random problem and checks
    // Advice:
    // structure the code so parameters can be easily tuned (hyperparameters). Could you give an example?
    // need to override the PlanMethod
    // need to reset member variables -> use the second grade function
    // gradient descent is somewhat easy. In the quatrants the closest point is normal to the side. In the eight-rants, the closest point is a direct line to the vertices
    // split regions are orthogonal to sides
    // what is the difference between a constuctor and a function

}; // namespace amp