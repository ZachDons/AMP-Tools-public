#include "MyHW7.h"
#include "MySampleAlgo.h"
#include "AMPCore.h"

amp::Path GenericPRM::plan(const Eigen::VectorXd &init_state,
                           const Eigen::VectorXd &goal_state,
                           const amp::ConfigurationSpace &collision_checker)
{
    limits.lower_limits = collision_checker.lowerBounds();
    limits.upper_limits = collision_checker.upperBounds();
    LOG("dims: "<<collision_checker.dimension());
    Eigen::VectorXd rand_q = getRandomState();

}

Eigen::VectorXd GenericPRM::getRandomState()
{
    std::size_t num_dim = limits.lower_limits.size();
    Eigen::VectorXd q_state(num_dim);
    LOG("num dim: "<<num_dim);

    for (int i_dim = 0; i_dim < num_dim; i_dim++)
    {
        LOG("ith dim: "<<i_dim);
        LOG("test lim: "<<limits.lower_limits(i_dim));
        LOG("test lim: "<<limits.upper_limits(i_dim));
        double q_i = amp::RNG::srandd(limits.lower_limits(i_dim), limits.upper_limits(i_dim));
        LOG("state: "<<q_i);
        q_state.coeffRef(i_dim) = q_i;

    }
    LOG("state:"<<q_state);
    return q_state;
    
}