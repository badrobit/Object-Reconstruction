/**
 * \file HelperFunctions.hpp
 * A container for commonly used functions for the object reconstruction program which do not belong
 * to just one piece of functionality. 
 */
#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

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

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/**
 *	A function that is used to write a standard PCL pointcloud to a PointCloud Data File (PCD).
 *
 *	@param file_name The file name as a string that you want the PCD file to have.
 *	@param input_cloud The PCL pointcloud that you want to save into an ASCII PCD file.
 *
 *	@return If we where able to verify if the point cloud was saved to the file properly or not.
 */
bool WriteToPCD( std::string file_name, PointCloud input_cloud )
{
  pcl::io::savePCDFileASCII( file_name + ".pcd", input_cloud );
  ROS_INFO_STREAM( "Saved " << input_cloud.size() << " datapoints to " << file_name << ".pcd" ); 
  return true; 
}

/** \brief This function is responsible for pre-processing the input pointcloud before we attempt 
 * object reconstruction. It will perform both a statistical outlier removal which will take the 
 * current point cloud and remove any points that are to far removed from all of its neighbors 
 * (cleaning up noise). It will also perform subsamling in order to make the following computations 
 * more efficient. 
 *
 * @param input_cloud The point cloud which we want to perform the pre-processing on. 
 * 
 * @return A pre-processed (noise removed & subsampled) pointcloud.
 */
PointCloud PreparePointCloud( PointCloud input_cloud )
{
	PointCloud temp_cloud; 
	int initial_cloud_size = input_cloud.size(); 

	// -------------- Statistical Outlier Removal ------------------------------------------------//
	ros::Time outlier_removal_start = ros::Time::now();
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud( input_cloud.makeShared() );
	sor.setMeanK( 50 );
	sor.setStddevMulThresh( 1.0 );
	sor.filter( temp_cloud );
	ROS_INFO_STREAM( "Removed " << initial_cloud_size - temp_cloud.size() << " Points during statistical outlier removal" ); 
	ROS_INFO_STREAM( "Outlier Removal took " << ros::Time::now() - outlier_removal_start << " seconds" ); 
	
	// -------------- Subsampling ----------------------------------------------------------------//
	ros::Time subsampling_start = ros::Time::now();
	pcl::VoxelGrid<PointT> subsample_filter;
	subsample_filter.setInputCloud( temp_cloud.makeShared() );
	subsample_filter.setLeafSize( 0.01f, 0.01f, 0.01f );
	subsample_filter.filter( temp_cloud );
	ROS_INFO_STREAM( "Removed " << initial_cloud_size - temp_cloud.size() << " Points during subsampling" ); 
	ROS_INFO_STREAM( "Subsampling took " << ros::Time::now() - subsampling_start << " seconds" ); 

	return temp_cloud; 
}
#endif /* HELPER_FUNCTIONS_H_ */