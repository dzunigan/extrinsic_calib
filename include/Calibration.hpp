//STL
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <cstddef>

#include "ObservationPair.hpp"
#include "Parameters.hpp"
#include "Plane.hpp"

typedef std::pair<Plane, Plane> PlaneMatch;
typedef std::vector< PlaneMatch > Correspondences;

class Calibration
{

public:

    Calibration(const ParametersPtr &params);

    //Finds correspondences between planes from a pair of observations, and adds them for future calibration process.
    //Returns a std::vector of found plane mathces (std::pair<Plane, Plane>).
    Correspondences findCorrespondences(const ObservationPair &obs2, const PlanesList &p1, const PlanesList &p2, bool use_initial_est = true);

    Eigen::Matrix4f runCalibration();

//    Eigen::Matrix4f runCalibrationGN();
//    Eigen::Matrix4f runCalibrationLM();

    Eigen::Matrix4f runCalibrationGN(std::size_t id_i, std::size_t id_j, bool robust = true);
    Eigen::Matrix4f runCalibrationLM(std::size_t id_i, std::size_t id_j, bool robust = true);

    Eigen::Matrix4d Tini; //Initial estimation iterative solutions

    std::vector< Correspondences > all_matches;

    //TODO: tested only with two sensors
    //Returns linear index for all_matches given two sensor_ids
    inline std::size_t getPairId(std::size_t id_i, std::size_t id_j)
    {
        if (id_i > id_j)
            std::swap(id_i, id_j);

        assert(id_i < id_j);

        return (id_i*sensor_id.size() + id_j - (id_i + 1)*(id_i + 2)/2); //i*N + j - sum(k+1)_(k=0)^(i)
    }

    std::size_t outliers;

protected:

    bool verbose;

    float max_angle;    // Degrees
    float max_distance; // Meters

    float approx_rotation; // Degrees
    float approx_translation; // Meters

    std::vector< Eigen::Matrix4f > init_pose;
    std::map< std::string, std::size_t > sensor_id;

    std::string save_dir;

    std::vector<mrpt::poses::CPose3D> poses;
    std::vector<std::string> labels;

    Eigen::Matrix3f calibrateRotation(std::size_t id_i, std::size_t id_j);

    Eigen::Vector3f calibrateTranslation(std::size_t id_i, std::size_t id_j);

    double robustKernel( double e );
};
