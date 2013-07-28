/*
 * PointCloudAccumulator.h
 *
 *  Created on: Jun 23, 2013
 *      Author: badrobot
 */

#ifndef POINTCLOUDACCUMULATOR_H_
#define POINTCLOUDACCUMULATOR_H_

#include <sensor_msgs/PointCloud2.h>

#include <hbrs_object_reconstruction/AccumulatePointCloud.h>

#include <HelperFunctions.hpp>

class PointCloudAccumulator
{
public:

	PointCloudAccumulator();
	virtual ~PointCloudAccumulator();
  PointCloud AccumulatePointClouds( int accumulation_time );

private:

	void AlignClouds( const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,
			              PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false );

	void PointCloudCallback( const sensor_msgs::PointCloud2::ConstPtr &ros_cloud );

protected:

  int                 m_point_cloud_count; 
  std::string         m_frame_id;
  Eigen::Matrix4f     m_global_transform; 
  PointCloud          m_accumulated_cloud; 

  ros::NodeHandle     m_node_handler; 
  ros::Publisher      m_accumulated_point_cloud_publisher; 

};

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

#endif /* POINTCLOUDACCUMULATOR_H_ */
