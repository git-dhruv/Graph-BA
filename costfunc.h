#ifndef costfunc_H
#define costfunc_H

#define CERES_USE_CXX11_THREADS = 1

#include <iostream>
#include "ceres/ceres.h"
#include "rotation.h"
#include <Eigen/Dense>

class costfunc{
    public:
    double observed_x, observed_y;
    //Constructor
    costfunc(double observed_x, double observed_y){
        observed_x = observed_x;
        observed_y = observed_y;
    }
    /*
    The use of templating here allows Ceres to call CostFunctor::operator<T>(), 
    with T=double when just the value of the residual is needed, 
    and with a special type T=Jet when the Jacobians are needed
    */
    template <typename T>
    bool operator()(const T *const camera,
                    const T *const point,
                    T *residuals) const {
                    T predictions[2];
        CamProjectionWithDistortion(camera, point, predictions);
        residuals[0] = predictions[0] - T(observed_x);
        residuals[1] = predictions[1] - T(observed_y);

                        return 1;

    }

    //Project World Coordinates into pixel space
    template<typename T>
    static inline bool CamProjectionWithDistortion(const T *camera, const T *Pw, T *Pc){
        //Camera is a 9 dims array
        //[3 axis angle], [3 trans], [f, k1, k2]
        //Pw is 3D point and Pc is corresponding Pixel

        T p[3];
        //Things got ugly with conversion of Eigen to Ceres Jet Type with this approach//
        /*
        //Homogenous Coordinates of Pw
        Eigen::Matrix<T, 1,4> P;
        P << Pw[0],Pw[1],Pw[2], 1;

        //Get homogenous Translation Matrix of Camera pose
        auto H = GetHomogenousMatrix(camera);
        //Transform World Coordinates
        P = H*P.transpose();
        */

        //Textbook approach //                
        AngleAxisRotatePoint(camera, Pw, p);
        // camera[3,4,5] are the translation
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        

        //Normalize the pixels - Signs are changed because of inverse convention
        T xp = -p[0]/p[2];
        T yp = -p[1]/p[2];

        // Apply second and fourth order radial distortion
        const T &k1 = camera[7];
        const T &k2 = camera[8];

        T r2 = xp * xp + yp * yp;
        T distortion = T(1.0) + k1 * r2 + k2 * r2 * r2;

        const T &focal = camera[6];
        Pc[0] = focal * distortion * xp;
        Pc[1] = focal * distortion * yp;
        
        return 1;

    }

    // //We ultimately want to run this Cost Function. We need a operator defined above
    static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
        // <CostFunc, dim of residual, dim of x(camera), dim of y(point)>
        return (new ceres::AutoDiffCostFunction<costfunc, 2, 9, 3>(
            new costfunc(observed_x, observed_y)));
    }

};
#endif