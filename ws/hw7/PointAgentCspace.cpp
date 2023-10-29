#include "MyHW7.h"
#include "AMPCore.h"

bool PointAgentCspace::inCollision(const Eigen::VectorXd &state) const
{
    int num_obs = m_env.obstacles.size();
    Eigen::Vector2d state_2d = {state[0], state[1]};
    vector<amp::Obstacle2D> all_obstacles = m_env.obstacles;
    bool collision_at_state;
    for (int i_obs = 0; i_obs < num_obs; i_obs = i_obs++)
    {
        collision_at_state = isPointInPolygon(all_obstacles[i_obs].verticesCCW(), state_2d);
        if (collision_at_state)
        {
            return true;
        }
    }
    return false;
}

bool PointAgentCspace::isPointInPolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &q) const
{
    int numVertices = polygon.size();
    bool inside = false;

    for (int i = 0, j = numVertices - 1; i < numVertices; j = i++)
    {
        const Eigen::Vector2d &vertexI = polygon[i];
        const Eigen::Vector2d &vertexJ = polygon[j];

        // Check if the point is on the edge of the polygon.
        if ((vertexI(1) > q(1)) != (vertexJ(1) > q(1)) &&
            q(0) < (vertexJ(0) - vertexI(0)) * (q(1) - vertexI(1)) / (vertexJ(1) - vertexI(1)) + vertexI(0))
        {
            inside = !inside;
        }
    }

    return inside;
}