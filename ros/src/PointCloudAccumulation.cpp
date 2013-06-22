/*
 * PointCloudAccumulation.cpp
 *
 *  Created on: Jun 22, 2013
 *      Author: badrobit
 */

#include "PointCloudAccumulation.h"

PointCloudAccumulation::PointCloudAccumulation( ros::NodeHandle i_node_handle ): m_node_handler( i_node_handle )
{

}

PointCloudAccumulation::~PointCloudAccumulation()
{
}

bool
PointCloudAccumulation::AccumulatePointCloud( hbrs_object_reconstruction::AccumulatePointCloud::Request& request,
			 	 	 	 	 	   	   	   	   	    hbrs_object_reconstruction::AccumulatePointCloud::Response& response )
{
	ROS_INFO("Received [accumulate_tabletop_cloud] request.");

	// Start a subscriber to the RGBD sesnor and capturing point clouds for the duration specified
	// by the user request.
	ros::Subscriber point_cloud_subscriber =
				m_node_handler.subscribe("/camera/depth_registered/points", 1, &PointCloudAccumulation::PointCloudCallback, this);

	ros::Time start = ros::Time::now();
	while( ros::Time::now() < start + ros::Duration( request.accumulation_time ) && ros::ok())
	{
	  ros::spinOnce();
	}
	point_cloud_subscriber.shutdown();
	ROS_INFO_STREAM("Accumulated" << GetPointCloudCount() << " clouds in " <<  (ros::Time::now() - start).toSec() << "seconds." );

	// Now that we have our accumulated point cloud pack it into a new single point cloud and send
	// it back as a response.
	pcl::PointCloud< pcl::PointXYZ > final_cloud;
	final_cloud.header.frame_id = m_frame_id;
	final_cloud.header.stamp = ros::Time::now();
	GetAccumulatedPointCloud( final_cloud );
	pcl::toROSMsg( final_cloud, response.point_cloud);

	return false;
}

void
PointCloudAccumulation::PointCloudCallback( const sensor_msgs::PointCloud2::ConstPtr &ros_cloud )
{
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::fromROSMsg( *ros_cloud, *cloud );
	m_frame_id = cloud->header.frame_id = ros_cloud->header.frame_id;
	AddPointCloud( cloud );
}

void
PointCloudAccumulation::AddPointCloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud )
{
	m_octree->setOccupiedVoxelsAtPointsFromCloud( cloud );
 	m_cloud_count++;
}

void
PointCloudAccumulation::GetAccumulatedPointCloud( pcl::PointCloud<pcl::PointXYZ>& cloud )
{
	m_octree->getOccupiedVoxelCenters(cloud.points);
	cloud.width = cloud.points.size();
	cloud.height = 1;
}

void
PointCloudAccumulation::ResetAccumulatedPointCloud()
{
	m_octree = OctreeUPtr( new Octree( m_resolution ) );
	m_cloud_count = 0;
}

int
PointCloudAccumulation::GetPointCloudCount()
{
	return m_cloud_count;
}
