/**
 * \file object_reconstruction.cpp
 * \author Matthew S Roscoe
 * \date July 30th 2013
 * \copyright GNU Public License.
 */

#include <HelperFunctions.h>
#include <PointCloudAccumulator.h>
#include <ObjectCandidateExtractor.h>
#include <hbrs_object_reconstruction/FixOcclusion.h>

/**
 * \class object_reconstruction_node
 *
 * \brief This class is responisble for handeling everything that is related to the object
 * reconstruction.
 *
 * \details It is the public interface for all of the backend processing that is required. It
 * advertises the following services that the user are able to call:
 * <ul><li>FixOcclusions</li></ul>
 */
class object_reconstruction_node
{
public:

  /**
   * \brief Constructor method for the ObjectReconstructionNode
   *
   * \details This method creates all of the required connections for this ROS node as well as
   * starting all of the subcomponents that this functionality requires.
   */
  object_reconstruction_node()
  {
    m_output_directory = ros::package::getPath( "hbrs_object_reconstruction" );
    ROS_WARN_STREAM( "Current Directory set to: " << m_output_directory );

    m_point_cloud_accumulator = PointCloudAccumulator();
    m_object_candidate_extractor = ObjectCandidateExtractor();

    m_fix_occlusions_service = m_node_handler.advertiseService( "FixOcclusions", &object_reconstruction_node::FixOcclusions, this );
    ROS_INFO_STREAM( "Advertised [FixOcclusions] Service" );
  }

private:

  /**
   * \brief Function that creates the "state machine" that connects all of the components for the
   * Occluded Geometry Estimation system.
   *
   * @param request The ROS service request handle.
   * @param response The ROS service response that the caller requires.
   * @return If the overall service process was completed successfully.
   */
  bool FixOcclusions( hbrs_object_reconstruction::FixOcclusion::Request  &request,
                      hbrs_object_reconstruction::FixOcclusion::Response &response )
  {
      m_resulting_cloud = m_point_cloud_accumulator.AccumulatePointClouds( 3 );

      HelperFunctions::WriteToPCD( m_output_directory + "/" + "01-AccumulatedPointCloud", m_resulting_cloud );

      m_resulting_cloud = HelperFunctions::PreparePointCloud( m_resulting_cloud );

      HelperFunctions::WriteToPCD( m_output_directory + "/" + "02-PreProcessedPointCloud", m_resulting_cloud );

      m_object_candidates = m_object_candidate_extractor.ExtractCandidateObjects( m_resulting_cloud );

      m_object_candidate_extractor.PublishObjectCandidates( m_object_candidates );

      HelperFunctions::WriteMultipleToPCD( m_output_directory + "/" + "03-ObjectCandidate", m_object_candidates );

      response.success = true;
      return response.success;
  }

  /** \brief Directory to write the PCD files to */
  std::string 				m_output_directory;
  /** \brief List of object candidates that we need to work with */
  std::vector<PointCloud>   m_object_candidates;
  /** \brief The ROS node handler for the object reconstruction process */
  ros::NodeHandle           m_node_handler;
  /** \brief The ROS Service server that advertises any services that this node offers */
  ros::ServiceServer  		m_fix_occlusions_service;
  /** \brief A temporary point cloud that we move from one step to another to track changes */
  PointCloud 			    m_resulting_cloud;
  /** \brief Reference object for the PointCloudAccumulator */
  PointCloudAccumulator		m_point_cloud_accumulator;
  /** \brief Reference object for the ObjectCandidateExtractor */
  ObjectCandidateExtractor    m_object_candidate_extractor;
};

/**
 * \brief This is the main class for the object reconstruction. It is responsible for starting the
 * processing and launching the object reconstruction node.
 *
 * @param argc The input arguments as a std::string
 * @param argv The number of input arguments.
 * @return Overall program success status.
 */
int main(int argc, char **argv)
{
	/// Initialize this ROS node. 
	ros::init(argc, argv, "hbrs_object_reconstruction");
	/// Create an instance of the object reconstruction.
	object_reconstruction_node ros_node;
	/// Start the ROS processing for this node. 
	ros::spin();
	return 0;
}
