// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW5.h"

// Include any custom headers you created in your workspace
#include "MyGradientDescent.h"

using namespace amp;

void problem1()
{
    // Use WO1 from Exercise
    Problem2D problem = HW2::getWorkspace1();
    //zeta 0.5, d_star = 15
    // double d_star, double q_star, double epsilon, double eta, double zeta, double step_size
    MyGradientDescent robot(3, 1, 0.25, 0.5, 0.2, 0.05);
    Path2D path = robot.plan(problem);
    Visualizer::makeFigure(problem, path);
}
void problem2()
{
    // Use WO1 from Exercise
    Problem2D problem = HW2::getWorkspace2();
    // double d_star, double q_star, double epsilon, double eta, double zeta, double step_size
    MyGradientDescent robot(3, 1, 0.25, 0.5, 0.2, 0.05);
    Path2D path = robot.plan(problem);
    Visualizer::makeFigure(problem, path);
}

void problem3()
{
    // Use WO1 from Exercise
    Problem2D problem = HW5::getWorkspace1();
    MyGradientDescent robot(3, 1, 0.25, 1, 0.2, 0.05);
    Path2D path = robot.plan(problem);
    Visualizer::makeFigure(problem, path);
}

int main(int argc, char **argv)
{
    //problem1();
    //problem2();
    //problem3();

    //Visualizer::showFigures();

    HW5::grade<MyGradientDescent>("zachary.donovan@colorado.edu",argc,argv,3, 1, 0.25, 1, 0.2, 0.05);

    return 0;
}