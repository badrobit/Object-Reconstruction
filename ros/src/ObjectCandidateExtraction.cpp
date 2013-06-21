/*
 * ObjectCandidateExtraction.cpp
 *
 *  Created on: Jun 20, 2013
 *      Author: badrobit
 */

#include "ObjectCandidateExtraction.h"

ObjectCandidateExtraction::ObjectCandidateExtraction()
{
}

ObjectCandidateExtraction::~ObjectCandidateExtraction()
{
}

bool ObjectCandidateExtraction::ExtractCandidateObjects( pcl::PointCloud<pcl::PointXYZ> &input_point_cloud,
															                               pcl::PointCloud<pcl::PointXYZ> &object_candidates )
{
    ROS_INFO( "Starting object candidate extraction" );
    ROS_INFO( "Object Candidate Extraction: Input Point Cloud Size: %d", (int) input_point_cloud.size() );
    ros::Time start, finish;
    start = ros::Time::now();

    if( input_point_cloud.size() == 0 )
    {
      ROS_WARN( "Object Candidate Extraction: Empty Point Cloud Passed In" );
      return false;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
   	tree->setInputCloud( input_point_cloud.makeShared() );

   	std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance( 0.01 ); // 2cm
    ec.setMinClusterSize( 100 );
    ec.setMaxClusterSize( 25000 );
    ec.setSearchMethod( tree );
    ec.setInputCloud( input_point_cloud.makeShared() );
    ec.extract( cluster_indices );

  	if( (int)cluster_indices.size() == 0 )
    {
      ROS_WARN( "Object Candidate Extraction: No candidate objects could be found" );
      return false;
    }

  	for( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it )
  	{
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster( new pcl::PointCloud<pcl::PointXYZ> );
    	for( std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++ )
    	{
    		cloud_cluster->points.push_back( input_point_cloud.points[*pit] );
    	}

      cloud_cluster->width = cloud_cluster->points.size();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;
      cloud_cluster->header = input_point_cloud.header;

      object_candidates = *cloud_cluster;
    	//object_candidates.push_back( *cloud_cluster );
      ROS_INFO( "Object Candidate Extraction: Added an object candidate of size: %d", (int)cloud_cluster->size() );
    }

    cluster_indices.clear();
    finish = ros::Time::now();
    ROS_INFO("candidate_extraction_time=%lf", (finish.toSec() - start.toSec() ));
    ROS_INFO( "Object Candidate Extraction: Final number of object candidates: %d", (int)object_candidates.size() );
    return true;
}
