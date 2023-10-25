// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

// Include the header
#include "MyLinkManipulator2D.h"
#include "MyConfigurationSpace.h"


using namespace amp;
using std::vector, Eigen::Vector2d;

// void Problem_1(){
//      /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
//     Obstacle2D obstacle_1 = HW4::getEx1TriangleObstacle();
//     vector<Vector2d> robot_vertices = {Vector2d(0,0),Vector2d(1,2),Vector2d(0,2)};
//     vector<double> angle_rad = {0.0};
//     MyClass problem_1a;
//     vector<amp::Polygon> Polygon_1a = problem_1a.Create_Cobs(obstacle_1, robot_vertices, angle_rad);
//     Visualizer::makeFigure(Polygon_1a);

    
//     const int num_angles = 12;
//     const double twoPi = 2 * M_PI;
//     vector<double> angle_values;
    
//     for (int i_angle = 0; i_angle < num_angles; ++i_angle) {
//         double angle_b = i_angle * (twoPi / num_angles);
//         angle_values.push_back(angle_b);
//     }

//     MyClass problem_1b;
//     vector<amp::Polygon> Polygon_1b = problem_1b.Create_Cobs(obstacle_1, robot_vertices, angle_values);
//     Visualizer::makeFigure(Polygon_1b,angle_values);
// }

// void Problem_2a(){
//     Eigen::Vector2d base_location = {0.0, 0.0};
//     std::vector<double> link_len = {0.5, 1.0, 0.5};
//     MyLinkManipulator2D robot(base_location, link_len);
//     ManipulatorState state = {M_PI/6, M_PI/3, 7*M_PI/4};
//     Visualizer::makeFigure(robot, state);
// }

// void Problem_2b(){
//     Eigen::Vector2d base_location = {0.0, 0.0};
//     Eigen::Vector2d end_effector = {2.0, 0.0};
//     std::vector<double> link_len = {1.0, 0.5, 1.0};
//     MyLinkManipulator2D robot(base_location, link_len);
//     ManipulatorState state = robot.getConfigurationFromIK(end_effector);
//     Visualizer::makeFigure(robot, state);
// }

// void Problem_3a(){
//     //C-Space Params (SxS)
//     int num_cells = 180; //one cell per degree
//     double x0_min = 0; // lower bound
//     double x0_max = 2*M_PI; // lower bound
//     double x1_min = 0; // lower bound
//     double x1_max = 2*M_PI; // lower bound

//     // Workspace
//     Environment2D ex3_workspace = HW4::getEx3Workspace1();
//     std::vector<double> link_len = {1.0, 1.0};
//     Eigen::Vector2d base_location = {0.0, 0.0};
//     MyLinkManipulator2D robot(base_location, link_len);
//     ManipulatorState state = {M_PI/6, M_PI/3};
//     Visualizer::makeFigure(ex3_workspace, robot, state);
//     Visualizer::makeFigure(ex3_workspace.obstacles);

//     // C-Space
//     MyConfigurationSpace config_space(num_cells,num_cells,x0_min,x0_max,x1_min,x1_max);
//     //config_space(1,1) = true;
//     config_space.compute_Cspace(ex3_workspace.obstacles, robot);
//     // compute_Cspace -> for(operator(i,j)); operator(i,j)=t/f incollision
//     // (*this)(i,j) = inCollision
//     Visualizer::makeFigure(config_space);
    
// }

int main(int argc, char** argv) {
   
    //Problem_1();
    //Problem_2a();
    //Problem_2b();
    //Problem_3a();

    Visualizer::showFigures();
    //amp::RNG::seed(amp::RNG::randiUnbounded());

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}