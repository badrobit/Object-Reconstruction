/**
 * \headerfile ObjectCandidateExtractor.h
 * \date Jul 28, 2013
 * \author Matthew S Roscoe http://matroscoe.ca
 */

#ifndef OBJECTCANDIDATEEXTRACTOR_H_
#define OBJECTCANDIDATEEXTRACTOR_H_

#include <HelperFunctions.hpp>

/**
 * \class ObjectCandidateExtractor
 * This class is responsible for finding groups of points in a point cloud which could
 * represent real world objects.
 */
class ObjectCandidateExtractor
{
public:
  /**
   * \brief Responsible for initializing and setting up all resources that are required to perform
   * object candidate extraction.
   */
  ObjectCandidateExtractor();

  /**
   * \brief Standard destructor that is used to free up any memory or ROS connections that where
   * created.
   */
  virtual ~ObjectCandidateExtractor();

  /**
   * \brief This function takes in a PCL Point Cloud and finds any candidate objects contained in
   * the scene.
   *
   * @param input_cloud The PCL PointCloud that will be checked for candidate objects.
   * @return A Vector which contains all of the candidate objects.
   */
  std::vector< PointCloud > ExtractCandidateObjects( PointCloud input_cloud );

  /**
   * \brief Function that takes in multiple object candidate point clouds and creates a single
   * ros message for them.
   *
   * \details This function takes in a STD Vector of PCL Point Clouds where each cloud is a
   * candidate object. It will then combine them all into one single STD_MSGS::PointCloud2 message
   * that can be published through ROS.
   *
   * @param input_vector
   * @return
   */
  bool PublishObjectCandidates( std::vector< PointCloud > input_vector );

protected:
  ros::NodeHandle     m_node_handler;
  ros::Publisher      m_object_candidates_publisher;
};

#endif /* OBJECTCANDIDATEEXTRACTOR_H_ */
