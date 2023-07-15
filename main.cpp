#include "costfunc.h"
#include "rotation.h"
#include <Eigen/Dense>
#include <vector>

int main(){

    std::vector <float> camera  {0,0,0,0,0,0}  ;

    auto H = GetHomogenousMatrix(camera);
    std::cout << H << std::endl;

    return 1;
}
