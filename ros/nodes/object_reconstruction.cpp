#include <ros/ros.h>

int main(int argc, char **argv)
{
	/**
	 * Start up ROS and create our node handler.
	 */
	ros::init(argc, argv, "hbrs_object_reconstruction");
	ros::NodeHandle node_handler;

	/*
	 * Default close out for a C/C++ program.
	 */
	return 0;
}
