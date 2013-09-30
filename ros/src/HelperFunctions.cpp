/*
 * HelperFunctions.cpp
 *
 *  Created on: Jul 29, 2013
 *      Author: badrobot
 */

#include "HelperFunctions.h"

/**
 *  \brief A function that is used to write a standard PCL pointcloud to a PointCloud Data File (PCD).
 *
 *  @param file_name The file name as a string that you want the PCD file to have.
 *  @param input_cloud The PCL pointcloud that you want to save into an ASCII PCD file.
 *
 *  @return If we where able to verify if the point cloud was saved to the file properly or not.
 */
bool HelperFunctions::WriteToPCD( std::string file_name, PointCloud input_cloud )
{
  pcl::io::savePLYFileASCII( file_name + ".ply", input_cloud );
  ROS_INFO_STREAM( "Saved " << input_cloud.size() << " datapoints to " << file_name << ".ply" );
  return true;
}

/**
 * \brief Function that is used to write out multiple PointClouds to Multiple PCD Files.
 *
 * @param file_name Desired series of file names. Index will be appended to it.
 * @param input_clouds A std::vector that contains at least 2 PCL PointCloud.
 * @return If we where able to properly write to PCD files.
 */
bool HelperFunctions::WriteMultipleToPCD( std::string file_name, std::vector<PointCloud> input_clouds )
{
  if( (int)input_clouds.size() < 1 )
  {
    ROS_ERROR_STREAM( "You provided [WriteMultipleToPCD] with less than 1 point clouds" );
    ROS_ERROR_STREAM( "Use [WriteToPCD] Instead .... Killing current process." );
    return false;
  }
  else
  {
    for( int i = 0; i < (int)input_clouds.size(); i++ )
    {
      std::stringstream ss;
      ss << file_name << "_" << i << ".ply";
      std::string file_name_index = ss.str();
      pcl::io::savePLYFileASCII( file_name_index, input_clouds[i] );
      ROS_INFO_STREAM( "Saved " << input_clouds[i].size() << " datapoints to " << file_name << "-" << i << ".ply" );
    }
    return true;
  }
}

PCLMesh HelperFunctions::ConvertCloudToMesh( std::string file_name, PointCloud input_cloud )
{
  ROS_WARN_STREAM( "Starting PointCloud to Mesh Conversion" );
  PCLMesh output_mesh;

  ROS_WARN_STREAM( "Normal Estimation" );
  pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
  ne.setNumberOfThreads( 8 );
  ne.setInputCloud( input_cloud.makeShared() );
  ne.setRadiusSearch( 0.01 );
  Eigen::Vector4f centroid;
  compute3DCentroid( input_cloud, centroid );
  ne.setViewPoint( centroid[0], centroid[1], centroid[2] );

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals( new pcl::PointCloud<pcl::Normal>() );
  ne.compute( *cloud_normals );

  for (size_t i = 0; i < cloud_normals->size (); ++i)
  {
    cloud_normals->points[i].normal_x *= -1;
    cloud_normals->points[i].normal_y *= -1;
    cloud_normals->points[i].normal_z *= -1;
  }

  PointCloudWithNormals::Ptr cloud_smoothed_normals (new PointCloudWithNormals );
  concatenateFields( input_cloud, *cloud_normals, *cloud_smoothed_normals   );

  // Create search tree*
  pcl::search::KdTree<PointNormalT>::Ptr tree2 (new pcl::search::KdTree<PointNormalT>);
  tree2->setInputCloud( cloud_smoothed_normals );

  // Initialize objects
  pcl::GridProjection<PointNormalT> gbpolygon;
  PCLMesh triangles;

  // Set parameters
  gbpolygon.setResolution( 0.005 );
  gbpolygon.setPaddingSize( 3 );
  gbpolygon.setNearestNeighborNum( 100 );
  gbpolygon.setMaxBinarySearchLevel( 10 );

  // Get result
  gbpolygon.setInputCloud( cloud_smoothed_normals );
  gbpolygon.setSearchMethod( tree2 );
  gbpolygon.reconstruct( triangles );

  pcl::io::savePolygonFileSTL( file_name + "-mesh.stl", triangles );
  //pcl::io::saveVTKFile( file_name + "mesh.vtk", triangles );
  ROS_INFO_STREAM( "Mesh saved to: " << file_name << "-mesh.stl" );

  return triangles;
}

bool HelperFunctions::PublishMeshMarker( ros::Publisher mesh_publisher, std::string file_name )
{
  ROS_WARN_STREAM( "Starting Mesh Conversion & Publication" );
  visualization_msgs::Marker mesh_marker;

  mesh_marker.header.frame_id = "\base_link";
  mesh_marker.header.stamp = ros::Time();
  mesh_marker.id = 775;
  mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  //mesh_marker.mesh_use_embedded_materials = true;
  //mesh_marker.action = visualization_msgs::Marker::ADD;

  mesh_marker.mesh_resource = file_name + "mesh.stl";
  ROS_WARN_STREAM( "Loaded mesh from: " << mesh_marker.mesh_resource );
  mesh_publisher.publish( mesh_marker );
  return true;
}

std::string HelperFunctions::SetOutputDirectory()
{
  std::string output_directory;

  output_directory += ros::package::getPath( "hbrs_object_reconstruction" );
  output_directory += "/data/";

  time_t t = time(0);   // get time now
  struct tm* now = localtime( & t );
  std::stringstream ss;
  ss << (now->tm_year + 1900) << '-'
     << (now->tm_mon + 1) << '-'
     << now->tm_mday << '-'
     << now->tm_hour << 'h'
     << now->tm_min << 'm';

  output_directory += ss.str();

  boost::filesystem::path dir( output_directory );
  if( boost::filesystem::create_directory(dir) )
  {
      ROS_INFO_STREAM( "Data Output Dir Created." );
  }

  output_directory += "/";

  return output_directory;
}
