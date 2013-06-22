/*
 * PointCloudAccumulation.h
 *
 *  Created on: Jun 22, 2013
 *      Author: badrobit
 */

#ifndef POINTCLOUDACCUMULATION_H_
#define POINTCLOUDACCUMULATION_H_

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_base.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>

#include "hbrs_object_reconstruction/AccumulatePointCloud.h"

class PointCloudAccumulation
{
public:
	PointCloudAccumulation();
	virtual ~PointCloudAccumulation();

private:

	/**
	 * This function takes in an amount of time for which the user wants to accumulate the point cloud for. It will
	 * then accumulate the information from a RGBD sensor and then return both the accumulated RAW point cloud
	 * (no subsampling or nosie reduction) in the form of a ROS sensor_msgs::PointCloud for the user to use as they
	 * please.
	 */
	 bool AccumulatePointCloud( hbrs_object_reconstruction::AccumulatePointCloud::Request& request,
			 	 	 	 	 	   hbrs_object_reconstruction::AccumulatePointCloud::Response& response );

	 void addCloud( const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud );

	 typedef pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ> Octree;
	 typedef std::unique_ptr<Octree> OctreeUPtr;

protected:
	 int 						m_cloud_count;
	 double 					m_resolution;


};

#endif /* POINTCLOUDACCUMULATION_H_ */
