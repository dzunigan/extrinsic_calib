//STL
#include <iostream>
#include <string>

#include <cstdlib>

//MRPT
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CConfigFile.h>

//Eigen
#include <Eigen/Dense>

//Boost
#include <boost/filesystem.hpp>

#include "Calibration.hpp"
#include "RawlogHelper.hpp"
#include "ObservationPair.hpp"
#include "Parameters.hpp"
#include "Segmentation.hpp"

inline Eigen::Matrix4f fromCPose3D(const mrpt::poses::CPose3D &p)
{
    mrpt::math::CMatrixDouble44 m;
    p.getHomogeneousMatrix(m);

    Eigen::Matrix4f t(m.cast<float>());
    return t;
}

int main (int argc, char ** argv)
{
    /// Input config

    if (argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " <config file>" << std::endl;
        return -1;
    }

    //Load config paramters
    std::string config_file(argv[1]);
    if (!boost::filesystem::is_regular_file(config_file))
    {
        std::cerr << "Invalid config file: " << config_file << std::endl;
        return -1;
    }

    mrpt::utils::CConfigFile cfg(config_file);

    ParametersPtr params(new Parameters);
    params->rawlog_file = cfg.read_string("Global", "rawlog_file", "", true);
    params->output_dir = cfg.read_string("Global", "output_dir", "", true);
    params->max_time_diff = cfg.read_float("Global", "max_dt", 0.015f, false);
    params->verbose = cfg.read_bool("Global", "verbose", true, false);

    params->visualize = cfg.read_bool("Segmentation", "visualize", true, false);
    params->sigma_s = cfg.read_float("Segmentation", "sigma_s", 5.0f, false);
    params->sigma_r = cfg.read_float("Segmentation", "sigma_r", 0.005f, false);
    params->max_change_factor = cfg.read_float("Segmentation", "max_change_factor", 0.02f, false);
    params->smoothing_size = cfg.read_float("Segmentation", "smoothing_size", 10.0f, false);
    params->inliers_ratio = cfg.read_float("Segmentation", "inliers_ratio", 0.2f, false);
    params->angular_th = cfg.read_float("Segmentation", "angular_th", 0.10f, false);
    params->distance_th = cfg.read_float("Segmentation", "distance_th", 0.03f, false);

    params->max_angle = cfg.read_float("Calibration", "max_angle", 7.f, false);
    params->max_distance = cfg.read_float("Calibration", "max_distance", 0.03f, false);

    params->approx_rotation = cfg.read_float("Calibration", "approx_rotation", 10.f, false);
    params->approx_translation = cfg.read_float("calibration", "approx_translation", 0.05f, false);

    //int decimation = cfg.read_int("Global", "decimation", 1, false);
    //bool display = cfg.read_bool("Global", "display", true, false);

    std::vector< std::string > all_labels;
    cfg.getAllSections(all_labels);

    for (std::vector<std::string>::iterator it = all_labels.begin(); it != all_labels.end(); ++it)
    {
        if (cfg.read_bool(*it, std::string("is_sensor"), false))
        {
            mrpt::poses::CPose3D pose = mrpt::poses::CPose3D(
                        cfg.read_float(*it, "pose_x", 0, true),
                        cfg.read_float(*it, "pose_y", 0, true),
                        cfg.read_float(*it, "pose_z", 0, true),
                        mrpt::utils::DEG2RAD(cfg.read_float(*it, "pose_yaw", 0, true)),
                        mrpt::utils::DEG2RAD(cfg.read_float(*it, "pose_pitch", 0, true)),
                        mrpt::utils::DEG2RAD(cfg.read_float(*it, "pose_roll", 0, true))
                        );

            params->sensor_labels.push_back(*it);
            params->init_pose.push_back(fromCPose3D(pose));
        }
    }

    if (params->sensor_labels.size() <= 1)
    {
        std::cerr << "Config file must contain more than one sensor specification" << std::endl;
        return -1;
    }

    //TODO: remove 2 sensors only constraint
    if (params->sensor_labels.size() != 2)
    {
        std::cerr << "Allowing 2 sensors only..." << std::endl;
        return -1;
    }

    if (!boost::filesystem::exists(params->rawlog_file))
    {
        std::cerr << "Invalid rawlog file: " << params->rawlog_file << std::endl;
        return -1;
    }

    if (!boost::filesystem::is_directory(params->output_dir))
    {
        std::cerr << "Invalid ouput_dir in config file: " << params->output_dir << std::endl;
        return -1;
    }

    if (params->verbose)
    {
        std::cout << "Calibration config: " << std::endl;
        std::cout << "rawlog_file: " << params->rawlog_file << std::endl;
        std::cout << "output_dir: " << params->output_dir << std::endl;

        std::cout << "max_time_diff: " << params->max_time_diff << std::endl;

        std::cout << "visualize: " << params->visualize << std::endl;
        std::cout << "sigma_s: " << params->sigma_s << std::endl;
        std::cout << "sigma_r: " << params->sigma_r << std::endl;
        std::cout << "max_change_factor: " << params->max_change_factor << std::endl;
        std::cout << "smoothing_size: " << params->smoothing_size << std::endl;
        std::cout << "inliers_ratio: " << params->inliers_ratio << std::endl;
        std::cout << "angular_th: " << params->angular_th << std::endl;
        std::cout << "distance_th: " << params->distance_th << std::endl;

        std::cout << "max_angle: " << params->max_angle << std::endl;
        std::cout << "max_distance: " << params->max_distance << std::endl;
        std::cout << "approx_rotation: " << params->approx_rotation << std::endl;
        std::cout << "approx_translation: " << params->approx_translation << std::endl;

//        std::cout << "decimation: " << decimation << std::endl;
//        std::cout << "display: " << display << std::endl;

        std::cout << "Number of sensors: " << params->sensor_labels.size() << std::endl;

        std::cout << std::endl;
        std::cout << "Loading rawlog file..." << std::endl;
    }

    /// Plane Segmentation & Matching

    RawlogHelper rawlog(params);

    if (params->verbose)
        std::cout << " done." << std::endl;

    //output_path = boost::filesystem::path(output_dir);

    Segmentation seg(params);
    Calibration calib(params);

    int cntr = 0;

    ObservationPair obs2;
    while (rawlog.getNextPair(obs2))
    {
        cntr++;

        PlanesList p1, p2;
        seg.segmentPlanes(obs2.first, p1);
        seg.segmentPlanes(obs2.second, p2);

        if (params->verbose)
        {
            std::cout << "Number of planes " << obs2.first->sensorLabel << ": " << p1.size() << std::endl;
            std::cout << "Number of planes " << obs2.second->sensorLabel << ": " << p2.size() << std::endl;
        }

        Correspondences matches = calib.findCorrespondences(obs2, p1, p2, false);

        if (params->verbose)
            std::cout << "Found correspondences: " << matches.size() << std::endl;
    }

    if (params->verbose)
    {
        std::cout << "Observations: " << cntr << "/" << rawlog.m << std::endl;
        std::cout << "Matches: " << calib.all_matches[0].size() << "/" << cntr << std::endl;
        std::cout << "Outliers: " << calib.outliers << "/" << calib.all_matches[0].size() << std::endl;
    }

    /// Calibration

    Eigen::Matrix4f T_cf = calib.runCalibration();

    std::cout << "CF:" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            std::cout << T_cf(i, j) << ", ";
        }
        std::cout << std::endl;
    }

    calib.Tini = Eigen::Matrix4d::Identity();
    Eigen::Matrix4f T_lm = calib.runCalibrationLM(0, 1, false);

    std::cout << "LM:" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            std::cout << T_lm(i, j) << ", ";
        }
        std::cout << std::endl;
    }

    Eigen::Matrix4f T_lmr = calib.runCalibrationLM(0, 1, true);

    std::cout << "LMR:" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            std::cout << T_lmr(i, j) << ", ";
        }
        std::cout << std::endl;
    }

    return 0;
}
