//MRPT
#include <mrpt/obs/CObservation3DRangeScan.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Parameters.hpp"
#include "Plane.hpp"

class Segmentation
{

public:

    Segmentation(const ParametersPtr &params);

    void segmentPlanes(const mrpt::obs::CObservation3DRangeScanPtr &obs, PlanesList &planes) const;

protected:

    float sigma_s;
    float sigma_r;

    float max_change_factor;
    float smoothing_size;

    float inliers_ratio;
    float angular_th;
    float distance_th;

    bool verbose;

    bool visualize;
    pcl::visualization::PCLVisualizer::Ptr viewer;

};
