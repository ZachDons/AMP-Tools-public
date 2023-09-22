#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm
{
public:
    // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
    virtual amp::Path2D plan(const amp::Problem2D &problem) override;

    // Add any other methods here...
    double is_in_primative(Eigen::Vector2d vertex_1, Eigen::Vector2d vertex_2, Eigen::Vector2d bug_position);
    bool is_in_obstacles(const amp::Problem2D &problem, Eigen::Vector2d bug_position);
    bool all_elements_negative(Eigen::ArrayXd element_array, int num_elements);
    Eigen::Vector2d RotMat(Eigen::Vector2d r_before, double theta_rot);
    void TakeStep(Eigen::Vector2d unit_vec);
    void Rotate(double bug_rot_angle);
    void Circumnavigate(const amp::Problem2D &problem);
    void GoToLeavePoint(const amp::Problem2D &problem);
    //amp::Path2D Bug1(const amp::Problem2D &problem, const amp::Path2D &path)
    void CircumnavigateBug2(const amp::Problem2D &problem);
    void ReOrientBug2(const amp::Problem2D &problem);




private:
    // Add any member variables here...
 


    // member variables
    double bug_step_len;
    Eigen::Vector2d bug_position;
    Eigen::Vector2d front_tenna_pos; 
    Eigen::Vector2d right_tenna_pos;
    int path_index_obs_hit;
    double bug_rot_angle;

    double tenna_mag;
    double tenna_rot_angle;

    Eigen::Vector2d vec_init_to_goal;


    amp::Problem2D problem;
    amp::Path2D path;
    


};