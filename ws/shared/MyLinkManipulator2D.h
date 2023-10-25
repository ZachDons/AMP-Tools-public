#pragma once // includes are included only once
#include "AMPCore.h"
#include <cmath>
#include <iostream>
using std::vector, Eigen::Vector2d, std::cout;

using namespace amp;

class MyLinkManipulator2D : public LinkManipulator2D
{
public:
    MyLinkManipulator2D();
    MyLinkManipulator2D(const std::vector<double> &link_lengths);
    MyLinkManipulator2D(const Eigen::Vector2d &base_location, const std::vector<double> &link_lengths);
    Eigen::Vector2d getJointLocation(const ManipulatorState &state, uint32_t joint_index) const override;
    ManipulatorState getConfigurationFromIK(const Eigen::Vector2d &end_effector_location) const override;

}; // namespace amp