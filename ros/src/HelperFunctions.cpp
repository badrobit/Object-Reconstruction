/*
 * HelperFunctions.cpp
 *
 *  Created on: Jul 29, 2013
 *      Author: badrobot
 */

#include "HelperFunctions.h"

/**
 * \brief This function is responsible for pre-processing the input pointcloud before we attempt
 * object reconstruction.
 *
 * \details It will perform both a statistical outlier removal which will take the
 * current point cloud and remove any points that are to far removed from all of its neighbors
 * (cleaning up noise). It will also perform subsamling in order to make the following computations
 * more efficient.
 *
 * @param input_cloud The point cloud which we want to perform the pre-processing on.
 *
 * @return A pre-processed (noise removed & subsampled) pointcloud.
 */
PointCloud HelperFunctions::PreparePointCloud( PointCloud input_cloud )
{
    PointCloud temp_cloud;
    int cloud_size = input_cloud.size();

    // -------------- Statistical Outlier Removal ------------------------------------------------//
    ros::Time outlier_removal_start = ros::Time::now();
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud( input_cloud.makeShared() );
    sor.setMeanK( 50 );
    sor.setStddevMulThresh( 1.0 );
    sor.filter( temp_cloud );
    ROS_INFO_STREAM( "Removed " << cloud_size - temp_cloud.size() << " Points during statistical outlier removal" );
    ROS_INFO_STREAM( "Outlier Removal took " << ros::Time::now() - outlier_removal_start << " seconds" );

    // -------------- Region of Interest Removal -------------------------------------------------//
    cloud_size = temp_cloud.size();
    ros::Time roi_start = ros::Time::now();

    pcl::PassThrough<PointT> passthrough_filter;
    passthrough_filter.setInputCloud( temp_cloud.makeShared() );
    passthrough_filter.setFilterFieldName( "z" );
    passthrough_filter.setFilterLimits( 0.60, 1.00 );
    passthrough_filter.filter( temp_cloud );

    ROS_INFO_STREAM( "Removed " << cloud_size - temp_cloud.size() << " Points during ROI" );
    ROS_INFO_STREAM( "ROI took " << ros::Time::now() - roi_start << " seconds" );

    // -------------- Subsampling ----------------------------------------------------------------//
    cloud_size = temp_cloud.size();
    ros::Time subsampling_start = ros::Time::now();
    pcl::VoxelGrid<PointT> subsample_filter;
    subsample_filter.setInputCloud( temp_cloud.makeShared() );
    subsample_filter.setLeafSize( 0.01f, 0.01f, 0.01f );
    subsample_filter.filter( temp_cloud );
    ROS_INFO_STREAM( "Removed " << cloud_size - temp_cloud.size() << " Points during subsampling" );
    ROS_INFO_STREAM( "Subsampling took " << ros::Time::now() - subsampling_start << " seconds" );

    return temp_cloud;
}

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
  pcl::io::savePCDFileASCII( file_name + ".pcd", input_cloud );
  ROS_INFO_STREAM( "Saved " << input_cloud.size() << " datapoints to " << file_name << ".pcd" );
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
      ss << file_name << "_" << i << ".pcd";
      std::string file_name_index = ss.str();
      pcl::io::savePCDFileASCII( file_name_index, input_clouds[i] );
      ROS_INFO_STREAM( "Saved " << input_clouds[i].size() << " datapoints to " << file_name << "_" << i << ".pcd" );
    }
    return true;
  }
}

PCLMesh HelperFunctions::ConvertCloudToMesh( std::string file_name, PointCloud input_cloud )
{
  ROS_WARN_STREAM( "Starting PointCloud to Mesh Conversion" );
  PCLMesh output_mesh;

  ROS_WARN_STREAM( "MLS Processing" );
  pcl::MovingLeastSquaresOMP<PointT, PointT> mls;
  mls.setInputCloud( input_cloud.makeShared() );
  mls.setSearchRadius( 0.01 );
  mls.setPolynomialFit( true );
  mls.setPolynomialOrder( 2 );
  mls.setUpsamplingMethod( pcl::MovingLeastSquaresOMP<PointT, PointT>::VOXEL_GRID_DILATION );
  mls.setUpsamplingRadius( 0.005 );
  mls.setUpsamplingStepSize( 0.003 );
  mls.setDilationVoxelSize( 0.001 );

  PointCloud cloud_smoothed;
  mls.process( cloud_smoothed );
  WriteToPCD( file_name + "04-Upscaling", cloud_smoothed );

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

  pcl::io::savePolygonFileSTL( file_name + "mesh.stl", triangles );
  //pcl::io::saveVTKFile( file_name + "mesh.vtk", triangles );
  ROS_INFO_STREAM( "Mesh saved to: " << file_name << "mesh.stl" );

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

bool HelperFunctions::ComputeBoundary( PointCloud input_cloud )
{
  PointCloud cloud_hull;
  PCLMesh temp_mesh;
  pcl::ConvexHull< PointT > hull;
  hull.setInputCloud( input_cloud.makeShared() );
  hull.setDimension( 3 );
  hull.reconstruct ( cloud_hull );

  if( cloud_hull.points.size() == 0 )
  {
    ROS_ERROR( "No points in reconstructed hull " );
    return false;
  }
  else
  {
    temp_mesh = ConvertCloudToMesh( "TEST", cloud_hull );

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh( temp_mesh );

    for( int i = 0; i < cloud_hull.points.size()-1; i++ )
    {
      std::stringstream ss;
      ss << "line" << i;
      viewer->addLine( cloud_hull.points[i], cloud_hull.points[i+1], ss.str() );
    }

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return true;
  }
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
