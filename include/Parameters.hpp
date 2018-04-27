#ifndef Z_PARAMETERS_HPP
#define Z_PARAMETERS_HPP

//STL
#include <string>
#include <vector>

//Eigen
#include <Eigen/Dense>

//Boost
#include <boost/smart_ptr/shared_ptr.hpp>

//Holds shared config parameter values
struct Parameters
{
    //Display
    bool verbose;

    //Synchronization
    float max_time_diff;

    //Segmentation
    bool visualize;

    float sigma_s;
    float sigma_r;

    float max_change_factor;
    float smoothing_size;

    float inliers_ratio;
    float angular_th;
    float distance_th;

    //Calibration
    float max_angle;
    float max_distance;

    float approx_rotation;
    float approx_translation;

    std::string rawlog_file;
    std::string output_dir;

    std::vector< std::string > sensor_labels;
    std::vector< Eigen::Matrix4f > init_pose;
};

typedef boost::shared_ptr<Parameters> ParametersPtr;

#endif
