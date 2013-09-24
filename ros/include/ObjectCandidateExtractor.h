/**
 * \headerfile PointCloudAccumulator.h
 * \author Matthew S Roscoe
 * \date July 30th 2013
 * \copyright GNU Public License.
 */

#ifndef OBJECTCANDIDATEEXTRACTOR_H_
#define OBJECTCANDIDATEEXTRACTOR_H_

#include <HelperFunctions.h>

/**
 * \class ObjectCandidateExtractor
 *
 * \brief This class is responsible for finding groups of points in a point cloud which could
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

  PointCloud RestorePlaneInteraction( pcl::ModelCoefficients::Ptr i_coefficients, PointCloud i_model );

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
  /** \brief Node Handler for the \ref ObjectCandidateExtractor class */
  ros::NodeHandle   m_node_handler;
  /** \brief A ROS publisher for the compiled object candidates */
  ros::Publisher    m_object_candidates_publisher;
};

#endif /* OBJECTCANDIDATEEXTRACTOR_H_ */
