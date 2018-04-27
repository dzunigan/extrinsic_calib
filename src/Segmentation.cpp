#include "Segmentation.hpp"

//STL
#include <cstddef>

//MRPT
#include <mrpt/math/CMatrix.h>

//PCL
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

inline void projectTo(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const mrpt::obs::CObservation3DRangeScanPtr &obs)
{
    const mrpt::math::CMatrix range = obs->rangeImage;

    float inv_fx = 1.0f / obs->cameraParams.fx();
    float inv_fy = 1.0f / obs->cameraParams.fy();
    float cx = static_cast<float> (obs->cameraParams.cx());
    float cy = static_cast<float> (obs->cameraParams.cy());

    unsigned int width = range.cols();
    unsigned int height = range.rows();

    cloud->clear();
    cloud->width = width;
    cloud->height = height;
    cloud->resize(width*height);

//    float max = std::numeric_limits<float>::min();
//    float min = std::numeric_limits<float>::max();

    cloud->is_dense = false;
    for (unsigned int v = 0; v < height; ++v)
    {
        for (unsigned int u = 0; u < width; ++u)
        {
            const float Z = range(v, u);

//            if (Z > max)
//                max = Z;
//            if (Z < min)
//                min = Z;

            //Setting unseen to 0 (instead of NaN) as bilateral filter set them to local max (may be undesirable)
            cloud->points[u+v*width].x = (u - cx)*Z*inv_fx;
            cloud->points[u+v*width].y = (v - cy)*Z*inv_fy;
            cloud->points[u+v*width].z = Z;
        }
    }

    //TODO: debug only
    //Check for min == 0 and isfinite(max)
//    std::cout << "Max: " << max << std::endl;
//    std::cout << "Min: " << min << std::endl;
}

Segmentation::Segmentation(const ParametersPtr &params)
{
    sigma_s = params->sigma_s;
    sigma_r = params->sigma_r;

    max_change_factor = params->max_change_factor;
    smoothing_size = params->smoothing_size;

    inliers_ratio = params->inliers_ratio;
    angular_th = params->angular_th;
    distance_th = params->distance_th;

    verbose = params->verbose;

    visualize = params->visualize;

    if (visualize)
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Cloud viewer"));
        viewer->initCameraParameters();
    }
}

void Segmentation::segmentPlanes(const mrpt::obs::CObservation3DRangeScanPtr &obs, PlanesList &planes) const
{
    //2. Get planes

    //2.1 Get point cloud
    //2.1.1 Compute 3D point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    projectTo(cloud, obs);

    //2.1.2 Filter point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::FastBilateralFilter<pcl::PointXYZ> bf;
    bf.setSigmaS(sigma_s);
    bf.setSigmaR(sigma_r);
    bf.setInputCloud(cloud);
    bf.filter(*filtered_cloud);

//    int v1;
//    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//    viewer.setBackgroundColor(1.0, 1.0, 1.0, v1);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 0, 0, 0);
//    viewer.addPointCloud(cloud, color1, "cloud1", v1);

//    int v2;
//    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//    viewer.setBackgroundColor(0, 0, 0, v2);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(filtered_cloud, 0,255, 0);
//    viewer.addPointCloud(filtered_cloud, color2, "cloud2", v2);

//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
//    viewer.addCoordinateSystem(0.1);

    if (visualize)
    {
        viewer->setBackgroundColor(1.0, 1.0, 1.0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(filtered_cloud, 0, 0, 0);
        viewer->addPointCloud(filtered_cloud, cloud_color, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->addCoordinateSystem(0.1);
    }

    //    while (!viewer.wasStopped())
    //    {
    //        viewer.spinOnce(100);
    //        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    //    }
    //    viewer.close();

    //2.2. Segment planes
    //2.2.1 Compute normals
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    //ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
    //ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    //ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(max_change_factor);
    ne.setNormalSmoothingSize(smoothing_size);
    ne.setDepthDependentSmoothing(true);

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(filtered_cloud);
    ne.compute(*normal_cloud);

//    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (filtered_cloud, normal_cloud, 100, 0.02, "normals2", v2);
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals2");

    //2.2.2 Plane segmentation
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(filtered_cloud->size()*inliers_ratio);
    mps.setAngularThreshold(angular_th);
    mps.setDistanceThreshold(distance_th);
    mps.setInputNormals(normal_cloud);
    mps.setInputCloud(filtered_cloud);

    //Using segmentAndRefine produces a plane estimation with more error (undesired), but with more inliers (desired)
    std::vector< pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator< pcl::PlanarRegion<pcl::PointXYZ> > > regions;
//    mps.segment(regions);
    mps.segmentAndRefine(regions);

    //    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    //    std::vector<pcl::ModelCoefficients> model_coefficients;
    //    std::vector<pcl::PointIndices> inliers;
    //    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    //    std::vector<pcl::PointIndices> label_indices;
    //    std::vector<pcl::PointIndices> boundary_indices;
    //    mps.segmentAndRefine(regions, model_coefficients, inliers, labels, label_indices, boundary_indices);

    planes.clear();
    planes.resize(regions.size());

    //2.2.3 Compute plane parameters
    for (std::size_t k = 0; k < regions.size(); ++k)
    {
        Eigen::Vector4f model = regions[k].getCoefficients();
        //model coefficients:
        //  model[0]: normal.x
        //  model[1]: normal.y
        //  model[2]: normal.z
        //  model[3]: d

        if (verbose)
            std::cout << "model: " << model.transpose() << std::endl;

        Eigen::Vector3f normal(model[0], model[1], model[2]);
        Eigen::Vector3f centroid = regions[k].getCentroid();

        //Note: the following step is unnessesary, but done here just to make sure all plane normals have the same sign.
        //Get normal pointing to origin (view point): n.dot(vp-centroid) > 0
        if (normal.dot(centroid) > 0)
          normal = -normal;

        planes[k].n = normal/normal.norm();
        planes[k].d = -planes[k].n.dot(centroid);

//        std::cout << "N: " << planes[k].n.transpose() << std::endl;
//        std::cout << "D: " << planes[k].d << std::endl;

        if (visualize)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            boundary_cloud->points = regions[k].getContour();

//            Warning: Showing only the last plane!!
//            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(boundary_cloud, 0, 255, 255);
//            viewer.addPointCloud(boundary_cloud, color, "plane", v2);
//            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "plane");

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundary_color(boundary_cloud, 0, 255, 255);
            viewer->addPointCloud(boundary_cloud, boundary_color, "plane");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "plane");

//            TODO: show plane normal

            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
            viewer->resetStoppedFlag();

            viewer->removePointCloud("plane");
        }
     }

    if (visualize) viewer->removePointCloud("cloud");
}
