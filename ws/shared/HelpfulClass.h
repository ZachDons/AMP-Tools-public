#include "AMPCore.h"

class MyClass
{
public:
    std::vector<Eigen::Vector2d> Minkoski_Sum(std::vector<Eigen::Vector2d> obstacle_vertices, std::vector<Eigen::Vector2d> robot_vertices);
    double Vector_Angle(Eigen::Vector2d point_1, Eigen::Vector2d point_2);
    std::vector<Eigen::Vector2d> Negative_Vertices(std::vector<Eigen::Vector2d> vertex_list);
    std::vector<Eigen::Vector2d> Sort_Vertices(std::vector<Eigen::Vector2d> vertex_list);
    Eigen::Vector2d Rotate_Vector(const Eigen::Vector2d& vec, double angle_rad);
    std::vector<Eigen::Vector2d> Rotate_Vertices(std::vector<Eigen::Vector2d> vertex_list, double angle_rad);
    std::vector<amp::Polygon> Create_Cobs(const amp::Obstacle2D &obstacle, std::vector<Eigen::Vector2d> robot_vertices, std::vector<double> angle_rad);

};