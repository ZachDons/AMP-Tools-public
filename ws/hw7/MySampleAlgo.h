#pragma once // includes are included only once
#include "AMPCore.h"
#include "hw/HW7.h"
// #include "MyLinkManipulator2D.h"
// #include "MyConfigurationSpace.h"
#include "ZachTools.h"
#include <cmath>
#include <algorithm>
#include <iostream>
using std::cout, std::vector, Eigen::Vector2d;

struct MyLimits
{
    Eigen::VectorXd lower_limits;
    Eigen::VectorXd upper_limits;
};

class GenericPRM
{
public:
    amp::Path plan(const Eigen::VectorXd &init_state,
                   const Eigen::VectorXd &goal_state,
                   const amp::ConfigurationSpace &collision_checker);
    Eigen::VectorXd getRandomState();

private:
    MyLimits limits;
};

class GenericRRT
{
public:
    amp::Path plan(const Eigen::VectorXd &init_state,
                   const Eigen::VectorXd &goal_state,
                   const amp::ConfigurationSpace &collision_checker);
};