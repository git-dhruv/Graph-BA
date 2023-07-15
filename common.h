#pragma once
#include <string>

class BALProblem {

private:

    int num_cameras_; 
    int num_points_; // no of 3D points in the scene
    int num_observations_; // no of observations. each observation has x,y coordinate
    int num_parameters_; // 6 for cameras, 3 for pnts

    int *point_index_; // pointer to 3D points    
    int *camera_index_; // pointer to cameras   
    double *observations_; // pointer to obs
    double *parameters_; // pointer to params

    // To convert camera center to angle, axis, and center
    void CameraToAngelAxisAndCenter(const double *camera,
                                    double *angle_axis,
                                    double *center) const;

    // To convert angle, axis, and center to camera center
    void AngleAxisAndCenterToCamera(const double *angle_axis,
                                    const double *center,
                                    double *camera) const;

public:
    // Load bal data from text file
    explicit BALProblem(const std::string &filename, bool use_quaternions = false);  

    //Destructor
    ~BALProblem() {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }

    //For export
    void WriteToPLYFile(const std::string &filename) const;

    // Compute the marginal median of the geometry
    void Normalize();

    // To purturb the data
    void Perturb(const double rotation_sigma,
                 const double translation_sigma,
                 const double point_sigma);

    // camera : 9 dims array
    // [0-2] : angle-axis rotation
    // [3-5] : translation
    // [6-8] : camera parameter, [6] focal length, [7-8] second and forth order radial distortion: u(1 + k1*r^2 + k2^r^4)
    int camera_block_size() const { return 9; }

    // 3D points
    int point_block_size() const { return 3; }

    int num_cameras() const { return num_cameras_; }

    int num_points() const { return num_points_; }

    int num_observations() const { return num_observations_; }

    int num_parameters() const { return num_parameters_; }

    const int *point_index() const { return point_index_; }

    const int *camera_index() const { return camera_index_; }

    const double *observations() const { return observations_; }

    const double *cameras() const { return parameters_; }

    double *mutable_cameras() { return parameters_; }

    double *mutable_points() { return parameters_ + camera_block_size() * num_cameras_; }
};
