#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "common.h"
#include "rotation.h"
#include "random.h"

typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;

template<typename T>
//////////////////////////////////////////////
// SKELETON CODE FUNCTIONS THAT ARE FOR I/O //
void FscanfOrDie(FILE *fptr, const char *format, T *value) { // cannot change the value at format
    // scanning data file
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1)
        std::cerr << "Invalid UW data file. ";
}

void PerturbPoint3(const double sigma, double *point) {
    // Adding noise to data (purturbing)
    for (int i = 0; i < 3; ++i)
        point[i] += RandNormal() * sigma;
}

double Median(std::vector<double> *data) {
    // use in normalizing the dataset
    int n = data->size();
    std::vector<double>::iterator mid_point = data->begin() + n / 2;
    std::nth_element(data->begin(), mid_point, data->end());
    return *mid_point;
}

void BALProblem::WriteToPLYFile(const std::string &filename) const {
    std::ofstream of(filename.c_str());

    of << "ply"
       << '\n' << "format ascii 1.0"
       << '\n' << "element vertex " << num_cameras_ + num_points_
       << '\n' << "property float x"
       << '\n' << "property float y"
       << '\n' << "property float z"
       << '\n' << "property uchar red"
       << '\n' << "property uchar green"
       << '\n' << "property uchar blue"
       << '\n' << "end_header" << std::endl;

    // Export extrinsic data (i.e. camera centers) as green points.
    double angle_axis[3];
    double center[3];
    for (int i = 0; i < num_cameras(); ++i) {
        const double *camera = cameras() + camera_block_size() * i;
        CameraToAngelAxisAndCenter(camera, angle_axis, center);
        of << center[0] << ' ' << center[1] << ' ' << center[2]
           << " 0 255 0" << '\n';
    }

    // Export the structure (i.e. 3D Points) as white points.
    const double *points = parameters_ + camera_block_size() * num_cameras_;
    for (int i = 0; i < num_points(); ++i) {
        const double *point = points + i * point_block_size();
        for (int j = 0; j < point_block_size(); ++j) {
            of << point[j] << ' ';
        }
        of << " 255 255 255\n";
    }
    of.close();
}

BALProblem::BALProblem(const std::string &filename, bool use_quaternions) {
    // constructor
    FILE *fptr = fopen(filename.c_str(), "r");

    if (fptr == NULL) {
        std::cerr << "Error: unable to open file " << filename;
        return;
    };

    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);

    std::cout << "Header: " << num_cameras_
              << " " << num_points_
              << " " << num_observations_;

    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i) {
        FscanfOrDie(fptr, "%d", camera_index_ + i);
        FscanfOrDie(fptr, "%d", point_index_ + i);
        for (int j = 0; j < 2; ++j) {
            FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
        }
    }

    for (int i = 0; i < num_parameters_; ++i) {
        FscanfOrDie(fptr, "%lf", parameters_ + i);
    }

    fclose(fptr);
}

//////////////////////////////////////////////

///////////////////Actual Math///////////////////////////
void BALProblem::Normalize() {
    /*
    Normalize the position of points and camera such that they are in the center of coordinate system
    This is just so that we remove the scale and bias in the data

    We first normalize 3D points using a median. 
    Using the same median, we normalize the Camera centers
    */
   
    //We will store single Axis data in this
    std::vector<double> singleAxis(num_points_);
    Eigen::Vector3d median;
    //This is a pointer that points to our loaded points
    double *points = mutable_points();

    //Compute Median along each axis
    for(size_t i=0;i<3;i++){
        for(size_t j=0;j<num_points_;j++){
            tmp.at(j) = points[3*j+i];
        }
        median(i) = Median(&tmp);
    }

    //Compute the MAD of 3D points
    //Equivalent of np.linalg.norm(points - median)
    for (size_t i = 0; i < num_points_; ++i) {
        VectorRef point(points + 3 * i, 3);
        tmp[i] = (point - median).lpNorm<1>();
    }

    //MAD
    const double median_absolute_deviation = Median(&tmp);

    //Scale so that the MAD is 100
    const double scale = 100.0 / median_absolute_deviation;

    // Normalize the 3D points 
    for (int i = 0; i < num_points_; ++i) {
        VectorRef point(points + 3 * i, 3);
        point = scale * (point - median);
    }

    //Uptil now we normalized the 3D points. now we normalize the camera position
    //We just need to change the center coordinates of the camera
    double *cameras = mutable_cameras();
    double angle_axis[3];
    double center[3];
    for (size_t i = 0; i < num_cameras_; ++i) {
        //Get single camera array
        double *camera = cameras + camera_block_size() * i;
        //Get the pose and location of camera
        CameraToAngelAxisAndCenter(camera, angle_axis, center);
        // Change the camera center = scale * (center - median)
        VectorRef(center, 3) = scale * (VectorRef(center, 3) - median);
        //Add that to the camera array
        AngleAxisAndCenterToCamera(angle_axis, center, camera);
    }
}