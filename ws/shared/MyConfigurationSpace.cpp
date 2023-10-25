#include "MyConfigurationSpace.h"
#include "MyLinkManipulator2D.h"
#include "ZachTools.h"
using namespace amp;

MyConfigurationSpace::MyConfigurationSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
    : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {}

// void MyConfigurationSpace::compute_Cspace(const vector<Obstacle2D> &obstacles, const MyLinkManipulator2D &robot)
// {
//     // Initialize grid size
//     std::pair<int, int> grid_size = size();
//     int num_obs = obstacles.size();
//     std::size_t num_links = robot.nLinks();

//     int x0_size = grid_size.first;
//     int x1_size = grid_size.second;
//     std::pair<int, int> i_x;

//     Vector2d q_i;
//     Vector2d p1;
//     Vector2d p2;
//     bool collision_at_state;

//     for (int i_x0 = 0; i_x0 < x0_size; i_x0++)
//     {
//         for (int i_x1 = 0; i_x1 < x1_size; i_x1++)
//         {
//             i_x = {i_x0, i_x1};
//             q_i = ZachTools::getPointFromIndex((*this), i_x);
//             LOG("State: " << q_i.x() << "," << q_i.y());


//             for (int i_obs = 0; i_obs < num_obs; i_obs++)
//             {
//                 collision_at_state = isChainIntersectingPolygon(obstacles[i_obs].verticesCCW(), robot,q_i);
//                 if (collision_at_state)
//                 {
//                     operator()(i_x0, i_x1) = 1;
//                     break;
//                 }
//             }
//         }
//     }
// }

void MyConfigurationSpace::compute_Cspace(const vector<Obstacle2D> &obstacles)
{

    // Initialize grid size
    std::pair<int, int> grid_size = size();
    int num_obs = obstacles.size();
    // std::pair<double, double> x0_lim = x0Bounds();
    // std::pair<double, double> x1_lim = x1Bounds();
    int x0_size = grid_size.first;
    int x1_size = grid_size.second;

    Vector2d q_i;
    bool collision_at_state;

    for (int i_x0 = 0; i_x0 < x0_size; i_x0++)
    {
        for (int i_x1 = 0; i_x1 < x1_size; i_x1++)
        {
            for (int i_obs = 0; i_obs < num_obs; i_obs++)
            {

                q_i = ZachTools::getPointFromIndex((*this), {i_x0, i_x1});
                // q_i = getPointFromIndex((*this),{i_x0, i_x1});
                collision_at_state = isPointInPolygon(obstacles[i_obs].verticesCCW(), q_i);

                if (collision_at_state)
                {
                    operator()(i_x0, i_x1) = 1;
                    break;
                }
            }
        }
    }
}

// bool MyConfigurationSpace::isChainIntersectingPolygon(const std::vector<Eigen::Vector2d> &polygon, const MyLinkManipulator2D &robot, const ManipulatorState &state)
// {
//     std::size_t num_links = robot.nLinks();
//     Vector2d p1;
//     Vector2d p2;
//     bool collision_at_state;
//     for (int i_link = 0; i_link < num_links - 1; i_link++)
//     {
//         p1 = MyLinkManipulator2D::getJointLocation(state, i_link);
//         p2 = MyLinkManipulator2D::getJointLocation(state, (i_link + 1));
//         collision_at_state = isLineIntersectingPolygon(polygon, p1, p2);
//         if (collision_at_state)
//         {
//             return true;
//         }
//     }
// }

// bool MyConfigurationSpace::inCollision(const Vector2d &q_i)
// {
//     int num_obs = all_obstacles.size();
//     bool collision_at_state;
//     for (int i_obs = 0; i_obs < num_obs; i_obs++)
//     {
//         collision_at_state = isPointInPolygon(all_obstacles[i_obs], q_i);
//         if (collision_at_state)
//         {
//             operator()(i_x0, i_x1) = 1;
//         }
//     }
// }

bool MyConfigurationSpace::isPointInPolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &q)
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

bool MyConfigurationSpace::isLineIntersectingPolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
{
    // Discretize the line segment
    int numSegments = 100; // Adjust the number of segments as needed for accuracy
    double deltaX = (p2(0) - p1(0)) / numSegments;
    double deltaY = (p2(1) - p1(1)) / numSegments;

    for (int i = 0; i <= numSegments; i++)
    {
        Eigen::Vector2d point(p1(0) + i * deltaX, p1(1) + i * deltaY);

        // Check if the point is in the polygon
        if (isPointInPolygon(polygon, point))
        {
            return true; // Intersection found
        }
    }

    return false; // No intersection found
}

std::pair<std::size_t, std::size_t> MyConfigurationSpace::getCellFromPoint(double x0, double x1) const
{
    return ZachTools::getCellFromPoint(*this, x0, x1);
}