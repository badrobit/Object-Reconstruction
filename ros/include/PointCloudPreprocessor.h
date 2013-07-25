/*
 * PointCloudPreprocessor.h
 *
 *  Created on: Jul 25, 2013
 *      Author: badrobot
 */

#ifndef POINTCLOUDPREPROCESSOR_H_
#define POINTCLOUDPREPROCESSOR_H_

#include <ros/ros.h>


class PointCloudPreprocessor
{
public:
  PointCloudPreprocessor();

  /**
   * Standard destructor.
   */
  virtual ~PointCloudPreprocessor();

private:

protected:
  ros::NodeHandle     m_node_handler;
};

#endif /* POINTCLOUDPREPROCESSOR_H_ */
