/** ObjectCandidateExtractor.cpp
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
}

std::vector< PointCloud >
ObjectCandidateExtractor::ExtractCandidateObjects( std::string file_name, PointCloud input_cloud )
{
  ROS_INFO_STREAM( "Starting to Extract Object Candidates" );
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
  plane_segmenter.setMaxIterations( 200 );
  plane_segmenter.setDistanceThreshold( 0.003 ); // 3mm

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

  pcl::search::KdTree< PointT >::Ptr tree (new pcl::search::KdTree< PointT >);
  tree->setInputCloud( input_cloud.makeShared() );

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance( 0.004 ); // 5mm
  ec.setMinClusterSize( 1000 );
  ec.setMaxClusterSize( 25000 );
  ec.setSearchMethod( tree );
  ec.setInputCloud( input_cloud.makeShared() );
  ec.extract( cluster_indices );

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloud::Ptr cloud_cluster (new PointCloud );
    cloud_cluster->header.frame_id = input_cloud.header.frame_id;
    for( std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++ )
    {
      cloud_cluster->points.push_back( input_cloud.points[*pit] );
    }

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    if( cloud_cluster->points.size() > 2000 )
    {
      ROS_INFO_STREAM( "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." );

      PointCloud complete;
      file_name += ("-" + to_string( j ) );
      complete = SmoothObjectCandidate( file_name, *cloud_cluster );
      complete = RestorePlaneInteraction( coefficients, complete );
      HelperFunctions::WriteToPCD( file_name + "-PlaneAdded", complete );

      if( complete.points.size() > 0 )
      {
        object_candidates.push_back( complete );
      }
      else
      {
        object_candidates.push_back( *cloud_cluster );
        ROS_WARN_STREAM( "Surface plane interaction not restored" );
      }
    }
    j++;
   }

  if( j == 0 )
  {
    ROS_ERROR( "No object candidates found aborting run" );
  }
  else
  {
    ROS_INFO_STREAM( j << " Object Candidates Found" );
  }

  return object_candidates;
}

PointCloud ObjectCandidateExtractor::RestorePlaneInteraction( pcl::ModelCoefficients::Ptr i_coefficients, PointCloud i_model )
{
  PointCloud CompleteModel;

  pcl::ProjectInliers<PointT> projection_filter;
  projection_filter.setModelType( pcl::SACMODEL_PLANE );
  projection_filter.setInputCloud( i_model.makeShared() );
  projection_filter.setModelCoefficients( i_coefficients );
  projection_filter.filter( CompleteModel );

  pcl::VoxelGrid<PointT> subsample_filter;
  subsample_filter.setInputCloud( CompleteModel.makeShared() );
  subsample_filter.setLeafSize( 0.002f, 0.002f, 0.002f );
  subsample_filter.filter( CompleteModel );

  if( CompleteModel.points.size() == 0 )
  {
    ROS_ERROR( "Projection Failed" );
  }

  // Add the object candidate to its projected plane.
  CompleteModel += i_model;

  return CompleteModel;
}

bool ObjectCandidateExtractor::PublishObjectCandidates( std::vector< PointCloud > input_vector )
{
  ROS_INFO_STREAM( "Publishing Object Candidates" );
  PointCloud output_cloud;

  // Go through all of the passed in pointclouds and add them to one large pointcloud.
  for( int i = 0; i < (int)input_vector.size(); i++ )
  {
    output_cloud += input_vector[i];
    ROS_INFO_STREAM( "Added " << input_vector[i].size() << " to the published candidate cloud" );
  }

  // Create the ROS message that we want to publish and provide it to anyone who is listening.
  sensor_msgs::PointCloud2 ros_output_cloud;
  pcl::toROSMsg( output_cloud, ros_output_cloud );
  ros_output_cloud.header.frame_id = input_vector[0].header.frame_id;
  m_object_candidates_publisher.publish( ros_output_cloud );

  return true;
}

PointCloud ObjectCandidateExtractor::SmoothObjectCandidate( std::string file_name, PointCloud inputCandidate )
{
    ROS_WARN_STREAM( "MLS Processing" );
    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setInputCloud( inputCandidate.makeShared() );
    mls.setSearchRadius( 0.005 );
    mls.setPolynomialFit( true );
    mls.setPolynomialOrder( 3 );
    mls.setUpsamplingMethod( pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION );
    mls.setUpsamplingRadius( 0.05 );
    mls.setUpsamplingStepSize( 0.003 );
    mls.setDilationVoxelSize( 0.005 );

    PointCloud cloud_smoothed;
    mls.process( cloud_smoothed );
    HelperFunctions::WriteToPCD( file_name + "-Smoothed", cloud_smoothed );

    return cloud_smoothed;
}
