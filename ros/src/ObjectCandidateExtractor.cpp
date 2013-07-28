/*
 * ObjectCandidateExtractor.cpp
 *
 *  Created on: Jul 28, 2013
 *      Author: badrobot
 */

#include "ObjectCandidateExtractor.h"

ObjectCandidateExtractor::ObjectCandidateExtractor()
{
  m_object_candidates_publisher = m_node_handler.advertise<sensor_msgs::PointCloud2> ( "ObjectCandidates", 1 );
  ROS_INFO_STREAM( "Now advertising [ObjectCandidates]");
}

ObjectCandidateExtractor::~ObjectCandidateExtractor()
{
  // TODO Auto-generated destructor stub
}

std::vector< PointCloud >
ObjectCandidateExtractor::ExtractCandidateObjects( PointCloud input_cloud )
{
  std::vector< PointCloud > object_candidates;

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation< PointT > plane_segmenter;
  pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
  pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );
  PointCloud::Ptr cloud_plane( new PointCloud() );
  PointCloud::Ptr cloud (new PointCloud ), cloud_f (new PointCloud );

  plane_segmenter.setOptimizeCoefficients( true );
  plane_segmenter.setModelType( pcl::SACMODEL_PLANE );
  plane_segmenter.setMethodType( pcl::SAC_RANSAC );
  plane_segmenter.setMaxIterations( 100 );
  plane_segmenter.setDistanceThreshold( 0.02 );

  int number_of_points = (int) input_cloud.points.size ();
  while( input_cloud.points.size () > 0.3 * number_of_points )
  {
    // Segment the largest planar component from the remaining cloud
    plane_segmenter.setInputCloud( input_cloud.makeShared() );
    plane_segmenter.segment( *inliers, *coefficients );
    if (inliers->indices.size () == 0)
    {
      ROS_ERROR_STREAM( "Could not find a plane model in the provided dataset" );
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices< PointT > extract;
    extract.setInputCloud( input_cloud.makeShared() );
    extract.setIndices( inliers );
    extract.setNegative( false );

    // Get the points associated with the planar surface
    extract.filter( *cloud_plane );
    ROS_WARN_STREAM( "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." );

    // Remove the planar inliers, extract the rest
    extract.setNegative( true );
    extract.filter( *cloud_f );
    input_cloud = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree< PointT >::Ptr tree (new pcl::search::KdTree< PointT >);
  tree->setInputCloud( input_cloud.makeShared() );

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction< PointT > ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud( input_cloud.makeShared() );
  ec.extract( cluster_indices );

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloud::Ptr cloud_cluster (new PointCloud );
    for( std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++ )
    {
      cloud_cluster->points.push_back( input_cloud.points[*pit] );
    }

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    ROS_INFO_STREAM( "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." );
    object_candidates.push_back( *cloud_cluster );
    j++;
  }

  return object_candidates;
}

bool ObjectCandidateExtractor::PublishObjectCandidates( std::vector< PointCloud > input_vector )
{
  PointCloud output_cloud;

  for( int i = 0; i < (int)input_vector.size(); i++ )
  {
    output_cloud += input_vector[i];
    ROS_INFO_STREAM( "Added " << input_vector[i].size() << " to the published candidate cloud" );
  }

  sensor_msgs::PointCloud2 ros_output_cloud;
  pcl::toROSMsg( output_cloud, ros_output_cloud );
  m_object_candidates_publisher.publish( ros_output_cloud );

  return true;
}
