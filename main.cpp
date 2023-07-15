#include "costfunc.h"
#include "rotation.h"
#include "common.h"
#include <Eigen/Dense>
#include <vector>

int main(int argc, char **argv){
    if (argc != 2) {
        cout << "usage: main bal_data.txt" << endl;
        return 1;
    }

    //This constructor will parse the data file and store the variables in appropriate vars
    BALProblem bal_problem(argv[1]);
    //We will normalize the data such that the point clouds are in center
    bal_problem.Normalize();
    //Perturbing the data so that we get some optimization problem to solve
    bal_problem.Perturb(0.1, 0.5, 1.5); 
    //Writing the perturbed file in initial pcl
    bal_problem.WriteToPLYFile("initial.ply");
    //Solving the Bundle Adjustment Problem
    SolveBA(bal_problem);
    //Saving the results
    bal_problem.WriteToPLYFile("final.ply");
    return 0;
}
