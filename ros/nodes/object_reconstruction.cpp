#include <HelperFunctions.hpp>
#include <PointCloudAccumulator.h>
#include <ObjectCandidateExtractor.h>
#include <hbrs_object_reconstruction/FixOcclusion.h>

/**
 *	This class is responisble for handeling everything that is related to the object reconstruction.
 * It is the public interface for all of the backend processing that is required. It advertises the 
 * following services that the user are able to call: 
 *
 * 	<ul>
 *		<li>FixOcclusions</li>
 *	</ul>
 */
class object_reconstruction_node
{
public:
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

	bool FixOcclusions( hbrs_object_reconstruction::FixOcclusion::Request  &request,
						hbrs_object_reconstruction::FixOcclusion::Response &response )
	{
		m_resulting_cloud = m_point_cloud_accumulator.AccumulatePointClouds( 3 ); 

		WriteToPCD( m_output_directory + "/" + "01-AccumulatedPointCloud", m_resulting_cloud ); 

		m_resulting_cloud = PreparePointCloud( m_resulting_cloud ); 

		WriteToPCD( m_output_directory + "/" + "02-PreProcessedPointCloud", m_resulting_cloud ); 

		m_object_candidates = m_object_candidate_extractor.ExtractCandidateObjects( m_resulting_cloud );

		m_object_candidate_extractor.PublishObjectCandidates( m_object_candidates );

		WriteMultipleToPCD( m_output_directory + "/" + "03-ObjectCandidate", m_object_candidates );

		response.success = true; 
		return response.success; 
	}

	std::string 				m_output_directory; 
	std::vector<PointCloud>     m_object_candidates;

  	ros::NodeHandle             m_node_handler;
	ros::ServiceServer  		m_fix_occlusions_service;

  	PointCloud 					m_resulting_cloud; 
  	PointCloudAccumulator		m_point_cloud_accumulator;
  	ObjectCandidateExtractor    m_object_candidate_extractor;
};


/**
 *  This is the main class for the object reconstruction. It is responsible for starting the
 * processing and launching the object reconstruction node. 
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
