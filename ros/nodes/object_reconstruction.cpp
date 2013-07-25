#include <HelperFunctions.hpp>

#include <PointCloudAccumulator.h>
#include <PointCloudPreprocessor.h>

#include <hbrs_object_reconstruction/FixOcclusion.h>

class object_reconstruction_node
{
public:
  object_reconstruction_node()
  {
  	m_output_directory = ros::package::getPath( "hbrs_object_reconstruction" );
  	ROS_WARN_STREAM( "Current Directory set to: " << m_output_directory ); 

  	m_point_cloud_accumulator = PointCloudAccumulator();
  	m_point_cloud_preprocessor = PointCloudPreprocessor(); 

  	m_fix_occlusions_service = m_node_handler.advertiseService( "FixOcclusions", &object_reconstruction_node::FixOcclusions, this );
	ROS_INFO_STREAM( "Advertised [FixOcclusions] Service" );
  }

private:

	bool FixOcclusions( hbrs_object_reconstruction::FixOcclusion::Request  &request,
						hbrs_object_reconstruction::FixOcclusion::Response &response )
	{
		m_resulting_cloud = m_point_cloud_accumulator.AccumulatePointClouds( 3 ); 

		WriteToPCD( m_output_directory + "/" + "AccumulatedPointCloud", m_resulting_cloud ); 

		return true; 
	}

	std::string 				m_output_directory; 

  	ros::NodeHandle             m_node_handler;
	ros::ServiceServer  		m_fix_occlusions_service;

  	PointCloud 					m_resulting_cloud; 

  	PointCloudAccumulator		m_point_cloud_accumulator; 
  	PointCloudPreprocessor      m_point_cloud_preprocessor;
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
