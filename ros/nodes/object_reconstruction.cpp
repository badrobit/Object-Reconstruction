#include <HelperFunctions.hpp>

#include <PointCloudAccumulator.h>

#include <hbrs_object_reconstruction/FixOcclusion.h>

class object_reconstruction_node
{
public:
	object_reconstruction_node()
	{
		m_output_directory = ros::package::getPath( "hbrs_object_reconstruction" );
		ROS_WARN_STREAM( "Current Directory set to: " << m_output_directory ); 

		m_point_cloud_accumulator = PointCloudAccumulator();

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

		response.success = true; 
		return response.success; 
	}

	std::string 				m_output_directory; 

  	ros::NodeHandle             m_node_handler;
	ros::ServiceServer  		m_fix_occlusions_service;

  	PointCloud 					m_resulting_cloud; 

  	PointCloudAccumulator		m_point_cloud_accumulator; 
};


/**
 *  This is the main class for the object reconstruction. It is responsible for starting the whole
 * process.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "hbrs_object_reconstruction");

	object_reconstruction_node ros_node;

	ros::spin();
	return 0;
}
