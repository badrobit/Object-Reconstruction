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

	 void AddPointCloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud );

	 void GetAccumulatedPointCloud( pcl::PointCloud<pcl::PointXYZ>& cloud );

	 void ResetAccumulatedPointCloud();

	 int GetPointCloudCount();

protected:

	 typedef pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ> Octree;
	 typedef std::unique_ptr<Octree> OctreeUPtr;

	 int 						m_cloud_count;
	 double 					m_resolution;
	 OctreeUPtr 				m_octree;


};

#endif /* POINTCLOUDACCUMULATION_H_ */
