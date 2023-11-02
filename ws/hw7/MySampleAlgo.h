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
    std::map<amp::Node, Eigen::Vector2d> getNode2Coord();
    amp::Graph<double> getCoordMap();
    GenericPRM(int _n, double _r, bool _smoothing);
    void createNode(const Eigen::VectorXd &state);
    void connectNode(const int &i_n);
    bool inLineCollision(const Eigen::VectorXd &p1, const Eigen::VectorXd &p2, const amp::ConfigurationSpace &collision_checker) const;
    void findRandomStates(const Eigen::VectorXd &init_state, const Eigen::VectorXd &goal_state,
                          const amp::ConfigurationSpace &collision_checker);
    void connectStates(const amp::ConfigurationSpace &collision_checker);

private:
    int n;
    double r;
    bool smoothing;
    MyLimits limits;
    amp::Node node_counter;
    amp::Graph<double> coord_map;
    std::vector<Eigen::VectorXd> all_states;
    std::map<amp::Node, Eigen::Vector2d> node_to_coord;
};

class GenericRRT
{
public:
    amp::Path plan(const Eigen::VectorXd &init_state,
                   const Eigen::VectorXd &goal_state,
                   const amp::ConfigurationSpace &collision_checker);
    Eigen::VectorXd getRandomState();
    std::map<amp::Node, Eigen::Vector2d> getNode2Coord();
    amp::Graph<double> getCoordMap();
    GenericRRT(int _n, double _r, double _p_goal, double _epsilon, bool _smoothing);
    void createNode(const Eigen::VectorXd &state, amp::Node &node_counter);
    std::pair<amp::Node, Eigen::VectorXd> getNearestToRand(const Eigen::VectorXd &rand_q, const amp::Node &node_counter);
    Eigen::VectorXd getNewState(const Eigen::VectorXd &q_nearest, const Eigen::VectorXd &q_rand) const;
    bool inLineCollision(const Eigen::VectorXd &p1, const Eigen::VectorXd &p2, const amp::ConfigurationSpace &collision_checker) const;
    void connectNode(const std::pair<amp::Node, Eigen::VectorXd> &q_nearest, const Eigen::VectorXd &q_new, const amp::Node &node_counter);
    


    //bool inSubPathCollision(const Eigen::VectorXd &q_nearest, const Eigen::VectorXd &q_rand, const amp::ConfigurationSpace &collision_checker) const;

    // void connectNode(const int &i_n);
    // void findRandomStates(const Eigen::VectorXd &init_state, const Eigen::VectorXd &goal_state,
    //                       const amp::ConfigurationSpace &collision_checker);
    // void connectStates(const amp::ConfigurationSpace &collision_checker);

private:
    int n;
    double r;
    double p_goal;
    double epsilon;
    bool smoothing;
    MyLimits limits;
    amp::Graph<double> coord_map;
    std::vector<Eigen::VectorXd> all_states;
    std::map<amp::Node, Eigen::Vector2d> node_to_coord;
};