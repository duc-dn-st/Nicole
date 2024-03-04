/*
 * ===============================================================================
 * test_surface_normal.cpp
 * Author: Schaefle Tobias
 * Date: 16.07.20
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * This file is just for testing and visualizing the cropped pointcloud and 
 * the surface normals.
 * ===============================================================================
 */

#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <eigen3/Eigen/Core>

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Input target.pcd" << std::endl;
        return 0;
    }

    std::string target_pcd = argv[1];

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(target_pcd, *target_cloud)) 
    {
        std::cerr << "failed to load " << target_pcd << std::endl;
        return 0;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> boxFilter_;
    // set crop values
    boxFilter_.setMin(Eigen::Vector4f(-3.0, -3.0, 0.0f, 1.0f));
    boxFilter_.setMax(Eigen::Vector4f(3.0, 3.0, 3.0f, 1.0f));
    boxFilter_.setInputCloud(target_cloud);
    boxFilter_.filter(*cropped_cloud);

    // Create the normal estimation class, and pass the input dataset to 
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cropped_cloud);
    // Create an empty kdtree representation, and pass it to the normal  estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.05);
    // Compute the features
    ne.compute(*cloud_normals);

/*
    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cropped_cloud, cloud_normals);

    while (!viewer.wasStopped())  // THE ORGINAL !viewer.wasStopped () 
    {
        viewer.spinOnce();
    }
*/
    return 0;
}