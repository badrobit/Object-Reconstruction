/*
 * HelperFunctions.cpp
 *
 *  Created on: Jul 29, 2013
 *      Author: badrobot
 */

#include <HelperFunctions.h>

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
PointCloud HelperFunctions::PreparePointCloud( PointCloud input_cloud )
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

/**
 *  \brief A function that is used to write a standard PCL pointcloud to a PointCloud Data File (PCD).
 *
 *  @param file_name The file name as a string that you want the PCD file to have.
 *  @param input_cloud The PCL pointcloud that you want to save into an ASCII PCD file.
 *
 *  @return If we where able to verify if the point cloud was saved to the file properly or not.
 */
bool HelperFunctions::WriteToPCD( std::string file_name, PointCloud input_cloud )
{
  pcl::io::savePCDFileASCII( file_name + ".pcd", input_cloud );
  ROS_INFO_STREAM( "Saved " << input_cloud.size() << " datapoints to " << file_name << ".pcd" );
  return true;
}

/**
 * \brief Function that is used to write out multiple PointClouds to Multiple PCD Files.
 *
 * @param file_name Desired series of file names. Index will be appended to it.
 * @param input_clouds A std::vector that contains at least 2 PCL PointCloud.
 * @return If we where able to properly write to PCD files.
 */
bool HelperFunctions::WriteMultipleToPCD( std::string file_name, std::vector<PointCloud> input_clouds )
{
  if( (int)input_clouds.size() < 2 )
  {
    ROS_ERROR_STREAM( "You provided [WriteMultipleToPCD] with less than 2 point clouds" );
    ROS_ERROR_STREAM( "Use [WriteToPCD] Instead .... Killing current process." );
    return false;
  }
  else
  {
    for( int i = 0; i < (int)input_clouds.size(); i++ )
    {
      std::stringstream ss;
      ss << file_name << "_" << i << ".pcd";
      std::string file_name_index = ss.str();
      pcl::io::savePCDFileASCII( file_name_index, input_clouds[i] );
      ROS_INFO_STREAM( "Saved " << input_clouds[i].size() << " datapoints to " << file_name << "_" << i << ".pcd" );
    }
    return true;
  }
}
