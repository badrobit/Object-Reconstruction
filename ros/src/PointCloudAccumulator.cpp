/*
 * PointCloudAccumulator.cpp
 *
 *  Created on: Jun 23, 2013
 *      Author: badrobot
 */

#include "PointCloudAccumulator.h"

PointCloudAccumulator::PointCloudAccumulator( ros::NodeHandle i_node_handle ): m_node_handler( i_node_handle )
{ 
	m_accumulate_point_clouds_service = m_node_handler.advertiseService( "AccumulatePointClouds", &PointCloudAccumulator::AccumulatePointClouds, this );
	ROS_INFO_STREAM( "Advertised [AccumulatePointClouds] Service" ); 

	m_accumulated_point_cloud_publisher = m_node_handler.advertise<sensor_msgs::PointCloud2> ( "AccumulatedPointCloud", 1 );
	ROS_INFO_STREAM( "Now advertising ");
}

PointCloudAccumulator::~PointCloudAccumulator()
{	
}

bool 
PointCloudAccumulator::AccumulatePointClouds( hbrs_object_reconstruction::AccumulatePointCloud::Request  &request,
											  hbrs_object_reconstruction::AccumulatePointCloud::Response &response )
{
	ROS_INFO_STREAM( "Recived [AccumulatePointCloud] Service Request" ); 

	m_point_cloud_count = 0; 
	m_global_transform = Eigen::Matrix4f::Identity();

	ROS_INFO_STREAM( "Starting polling from RGBD Sensor" );
    ros::Subscriber subscriber = 
    	m_node_handler.subscribe("/camera/depth_registered/points", 1, &PointCloudAccumulator::PointCloudCallback, this );

    ros::Time start = ros::Time::now();
    while( ros::Time::now() < start + ros::Duration( request.accumulation_time ) && ros::ok())
    {
      ros::spinOnce();
    }
    subscriber.shutdown();
    ROS_INFO_STREAM( "Shutting down RGBD Sensor subscriber" );
    ROS_WARN_STREAM( "Final accumulated PointCloud size: " << m_accumulated_cloud.size() ); 	

    pcl::toROSMsg( m_accumulated_cloud, response.point_cloud );
    m_accumulated_point_cloud_publisher.publish( response.point_cloud );

    ROS_INFO_STREAM( "PointCloud accumulation completed" ); 
	return true; 
}

void 
PointCloudAccumulator::PointCloudCallback( const sensor_msgs::PointCloud2::ConstPtr &ros_cloud )
{
	ROS_WARN_STREAM( "PointCloudCallback Started" ); 
	PointCloud::Ptr input_cloud( new PointCloud );
	pcl::fromROSMsg(*ros_cloud, *input_cloud);
	m_frame_id = ros_cloud->header.frame_id;
	m_accumulated_cloud.header.frame_id = m_frame_id;

	if( m_point_cloud_count == 0 )
	{
		pcl::copyPointCloud( *input_cloud, m_accumulated_cloud );
		ROS_WARN_STREAM( "Accumulated PointCloud size: " << m_accumulated_cloud.size() ); 		
		ROS_WARN_STREAM( "First PointCloud Added" ); 
		m_point_cloud_count += 1;
	}
	else
	{
		PointCloud::Ptr temp (new PointCloud);
		PointCloud::Ptr result( new PointCloud ); 
		Eigen::Matrix4f pairTransform;

		AlignClouds( input_cloud, m_accumulated_cloud.makeShared(), temp, pairTransform, true  );
		pcl::transformPointCloud( *temp, *result, m_global_transform );
		m_global_transform = pairTransform * m_global_transform;

		copyPointCloud( *result, m_accumulated_cloud ); 

		ROS_WARN_STREAM( "PointCloud " << m_point_cloud_count << " added to accumulated cloud" ); 
		m_point_cloud_count += 1; 
	}
}

void
PointCloudAccumulator::AlignClouds( const PointCloud::Ptr cloud_src,
									const PointCloud::Ptr cloud_tgt,
		                            PointCloud::Ptr output,
		                            Eigen::Matrix4f &final_transform,
		                            bool downsample )
{
	ROS_WARN_STREAM( "Starting PointCloud Allginment" ); 

	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud::Ptr src (new PointCloud);
	PointCloud::Ptr tgt (new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	if (downsample)
	{
		grid.setLeafSize (0.05, 0.05, 0.05);
		grid.setInputCloud (cloud_src);
		grid.filter (*src);

		grid.setInputCloud (cloud_tgt);
		grid.filter (*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}

	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (30);

	norm_est.setInputCloud (src);
	norm_est.compute (*points_with_normals_src);
	pcl::copyPointCloud (*src, *points_with_normals_src);

	norm_est.setInputCloud (tgt);
	norm_est.compute (*points_with_normals_tgt);
	pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);

	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon (1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (0.1);  
	// Set the point representation
	reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	reg.setInputCloud (points_with_normals_src);
	reg.setInputTarget (points_with_normals_tgt);

	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations (2);
	for (int i = 0; i < 30; ++i)
	{
		PCL_INFO ("Iteration Nr. %d.\n", i);

		// save ` for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputCloud (points_with_normals_src);
		reg.align (*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
		{
			reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
		}

		prev = reg.getLastIncrementalTransformation ();
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

	//add the source to the transformed target
	*output += *cloud_src;

  final_transform = targetToSource;
  ROS_WARN_STREAM( "PointCloud Alignment Completed" ); 
}
