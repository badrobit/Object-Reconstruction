/*
 * PointCloudAccumulator.h
 *
 *  Created on: Jun 23, 2013
 *      Author: badrobot
 */

#ifndef POINTCLOUDACCUMULATOR_H_
#define POINTCLOUDACCUMULATOR_H_

#include <ros/ros.h>

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

#include <hbrs_object_reconstruction/AccumulatePointCloud.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class PointCloudAccumulator
{
public:

	PointCloudAccumulator( ros::NodeHandle i_node_handle );
	virtual ~PointCloudAccumulator();

private:

	void AlignClouds( const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,
			              PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false );

	bool AccumulatePointClouds( hbrs_object_reconstruction::AccumulatePointCloud::Request  &request,
								              hbrs_object_reconstruction::AccumulatePointCloud::Response &response );

  void PointCloudCallback( const sensor_msgs::PointCloud2::ConstPtr &ros_cloud );

protected:

  int                 m_point_cloud_count; 
  std::string         m_frame_id;
  PointCloud::Ptr     m_accumulated_cloud; 
  Eigen::Matrix4f     m_global_transform; 

  ros::NodeHandle     m_node_handler; 
  ros::ServiceServer  m_accumulate_point_clouds_service; 
  ros::Publisher      m_accumulated_point_cloud_publisher; 

};

#endif /* POINTCLOUDACCUMULATOR_H_ */

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};
