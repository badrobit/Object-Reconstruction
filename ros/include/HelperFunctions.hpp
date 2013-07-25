#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <ros/ros.h>
#include <ros/package.h>

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

#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

bool WriteToPCD( std::string file_name, PointCloud input_cloud )
{
  pcl::io::savePCDFileASCII( file_name + ".pcd", input_cloud );
  ROS_INFO_STREAM( "Saved " << input_cloud.size() << " datapoints to " << file_name << ".pcd" ); 
  return true; 
}

#endif /* HELPER_FUNCTIONS_H_ */