#pragma once // includes are included only once
#include "AMPCore.h"
#include "hw/HW7.h"
#include "MySampleAlgo.h"
#include "ZachTools.h"
#include <cmath>
#include <algorithm>
#include <iostream>
using std::cout, std::vector, Eigen::Vector2d;

// 32
// 1c collision checker and distance metric and dimension change (because the space is no longer euclidean)

class MyPRM2D : public amp::PRM2D, public GenericPRM
{
public:
    MyPRM2D(int n, int r) : amp::PRM2D(), n(n), r(r) {}
    virtual amp::Path2D plan(const amp::Problem2D &problem) override;

private:
    int n;
    int r;
};

class MyRRT2D : public amp::GoalBiasRRT2D, public GenericRRT
{
public:
    MyRRT2D(int n, int r) : amp::GoalBiasRRT2D(), n(n), r(r) {}
    virtual amp::Path2D plan(const amp::Problem2D &problem) override;

private:
    int n;
    int r;
};
// << RRT

class PointAgentCspace : public amp::ConfigurationSpace
{
public:
    // PointAgentCspace(const amp::Environment2D &env)
    //     : ConfigurationSpace(Eigen::VectorXd{env.x_min, env.y_min}, Eigen::VectorXd{env.x_max, env.y_max}), m_env(env) {}
    PointAgentCspace(const amp::Environment2D &env)
        : ConfigurationSpace(Eigen::VectorXd(2), Eigen::VectorXd(2)), m_env(env)
    {
        m_lower_bounds[0] = env.x_min;
        m_upper_bounds[0] = env.x_max;
        m_lower_bounds[1] = env.y_min;
        m_upper_bounds[1] = env.y_max;
    }

    virtual bool inCollision(const Eigen::VectorXd &state) const override;
    bool isPointInPolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &q) const;

private:
    amp::Environment2D m_env;
};
