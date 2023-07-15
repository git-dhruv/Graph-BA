/*
@brief:
*/
#include "costfunc.h"
#include "common.h"
#include <Eigen/Dense>
#include <vector>
#include <ceres/ceres.h>

void SolveBA(BALProblem &bL);

int main(int argc, char **argv){
    if (argc != 2) {
        std::cout << "usage: main bal_data.txt" << std::endl;
        return 1;
    }

    //This constructor will parse the data file and store the variables in appropriate vars
    BALProblem bal_problem(argv[1]);
    //We will normalize the data such that the point clouds are in center
    bal_problem.Normalize();
    
    //Perturbing the data so that we get some optimization problem to solve
    bal_problem.Perturb(.1, 0.5, 0.5); 
    //Writing the perturbed file in initial pcl
    bal_problem.WriteToPLYFile("initial.ply");
    //Solving the Bundle Adjustment Problem
    SolveBA(bal_problem);
    //Saving the results
    bal_problem.WriteToPLYFile("final.ply");
    return 0;
}
void SolveBA(BALProblem &bal_problem) {
    const int point_block_size = bal_problem.point_block_size(); // 3
    const int camera_block_size = bal_problem.camera_block_size(); // 9
    double *points = bal_problem.mutable_points();
    double *cameras = bal_problem.mutable_cameras();

    //Corresponding Observation Pixels 2*N
    const double *observations = bal_problem.observations();
    ceres::Problem problem;

    //Construct a Ceres Problem
    for (size_t i = 0; i < bal_problem.num_observations(); ++i) {
        ceres::CostFunction *cost_function;

        //Remember we send x and y observation in the create function
        cost_function = costfunc::Create(observations[2 * i + 0], observations[2 * i + 1]);

        //We will use Huber Loss 
        ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0); 

        // Each observation corresponds to a pair of a camera and a point
        // which are identified by camera_index()[i] and point_index()[i]
        // respectively.
        double *camera = cameras + camera_block_size * bal_problem.camera_index()[i];
        double *point = points + point_block_size * bal_problem.point_index()[i];
        
        //add i−th residual into the problem
        // use auto−diff, template params: redisual type, output dimension, input dimension
        // shoule be same
        problem.AddResidualBlock(cost_function, loss_function, camera, point);
    }

    // show some information here
    std::cout << "bal problem file loaded..." << std::endl;
    std::cout << "bal problem have " << bal_problem.num_cameras() << " cameras and "
              << bal_problem.num_points() << " points. " << std::endl;
    std::cout << "Forming " << bal_problem.num_observations() << " observations. " << std::endl;

    std::cout << "Solving ceres BA ... " << std::endl;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}
