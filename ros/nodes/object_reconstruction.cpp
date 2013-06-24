#include <ros/ros.h>

#include <PointCloudAccumulator.h>


int main(int argc, char **argv)
{
	/**
	 * Start up ROS and create our node handler.
	 */
	ros::init(argc, argv, "hbrs_object_reconstruction");
	ros::NodeHandle node_handler;

	PointCloudAccumulator PCA( node_handler ); 

	ros::spin(); 

	/*
	 * Default close out for a C/C++ program.
	 */
	return 0;
}
