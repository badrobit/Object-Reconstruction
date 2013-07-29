/**
 * \headerfile HelperFunctions.h
 * \author Matthew S Roscoe
 * \date July 29th 2013
 * \copyright GNU Public License.
 */

#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_

#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

/**
 * \typedef PointT
 * \brief Have all points in the project represented by pcl::PointXYZRGB
 * */
typedef pcl::PointXYZRGB PointT;

/**
 * \typedef PointCloud
 * \brief Have all pointclouds used in the project made up from \ref PointT
 *
 * \details This representation is used to save time and type matching errors while passing around
 * and working with point clouds.
 */
typedef pcl::PointCloud<PointT> PointCloud;

/**
 * \typedef PointNormalT
 * \brief A standard PCL point that also includes Normal Information for that point.
 */
typedef pcl::PointNormal PointNormalT;

/**
 * \typedef PointCloudWithNormals
 * \brief A Standard PCL PointCloud but where the points all contain normal information as well.
 */
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/**
 * \class HelperFunctions
 * \brief This class is a grouping of functions that can be accessed from any
 */
class HelperFunctions
{
public:

  /**
   * \brief This function is responsible for pre-processing the input pointcloud before we attempt
   * object reconstruction.
   *
   * \details It will perform both a statistical outlier removal which will take the
   * current point cloud and remove any points that are to far removed from all of its neighbors
   * (cleaning up noise). It will also perform subsamling in order to make the following computations
   * more efficient.
   *
   * @param input_cloud The point cloud which we want to perform the pre-processing on.
   *
   * @return A pre-processed (noise removed & subsampled) pointcloud.
   */
  static PointCloud PreparePointCloud( PointCloud input_cloud );


  /**
   * \brief A function that is used to write a standard PCL pointcloud to a PointCloud Data File (PCD).
   *
   * @param file_name The file name as a string that you want the PCD file to have.
   * @param input_cloud The PCL pointcloud that you want to save into an ASCII PCD file.
   *
   * @return If we where able to verify if the point cloud was saved to the file properly or not.
   */
  static bool WriteToPCD( std::string file_name, PointCloud input_cloud );

  /**
   * \brief Function that is used to write out multiple PointClouds to Multiple PCD Files.
   *
   * @param file_name Desired series of file names. Index will be appended to it.
   * @param input_clouds A std::vector that contains at least 2 PCL PointCloud.
   * @return If we where able to properly write to PCD files.
   */
  static bool WriteMultipleToPCD( std::string file_name, std::vector<PointCloud> input_clouds );
};
#endif /* HELPERFUNCTIONS_H_ */
