#include <ros/ros.h>
#include <std_srvs/Empty.h>


int main(int argc, char **argv)
{
	/*
	 * Start up ROS and create our node handler.
	 */
	ros::init(argc, argv, "object_reconstruction");
	ros::NodeHandle node_handler;

	ros::spin();

	/*
	 * Default close out for a C/C++ program.
	 */
	return 0;
}
