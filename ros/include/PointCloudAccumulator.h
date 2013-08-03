/**
 * \headerfile PointCloudAccumulator.h
 * \author Matthew S Roscoe
 * \date July 30th 2013
 * \copyright GNU Public License.
 */

#ifndef POINTCLOUDACCUMULATOR_H_
#define POINTCLOUDACCUMULATOR_H_

#include <HelperFunctions.h>
//#include <hbrs_object_reconstruction/AccumulatePointCloud.h>

/**
 * \class PointCloudAccumulator
 * \brief This class takes two point clouds and merges them together.
 */
class PointCloudAccumulator
{
public:
  /**
   * \brief Standard Constructor.
   */
  PointCloudAccumulator();

  /**
   * \brief Standard Destructor.
   */
  virtual ~PointCloudAccumulator();

  /**
   * \brief Public interface that accumulates point clouds together.
   *
   * @param accumulation_time The amount of time that the PointClouds should be accumulated
   * for.
   *
   * @return A single PointCloud that contains all of the merged point clouds.
   */
  PointCloud AccumulatePointClouds( int accumulation_time );

  private:
  /**
   * \brief Takes in two PointClouds and merges them together.
   * \details
   *
   * @param cloud_src Input PointCloud to be merged into the cloud_tgt
   * @param cloud_tgt Master PointCloud that the cloud_src will be merged into.
   * @param output The two PointClouds merged together.
   * @param final_transform An Eigen tranformation matrix
   * @param downsample Do you want to downsample the source and target PointClouds
   */
  void AlignClouds( const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,
                    PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false );

  /**
   * \brief The function that is called every time that ROS gets a notification that there is a new
   * PointCloud available.
   *
   * @param ros_cloud The new PointCloud message that is being provided by ROS.
   */
  void PointCloudCallback( const sensor_msgs::PointCloud2::ConstPtr &ros_cloud );

protected:

  /** \brief Total number of point clouds that have been merged */
  int                 m_point_cloud_count; 
  /** \brief The reference FrameID to merge the PointCloud into */
  std::string         m_frame_id;
  /** \brief The global transformation matrix for moving the target cloud into the reference frame
   * of the source PointCloud */
  Eigen::Matrix4f     m_global_transform;
  /** \brief The resulting accumulated PointCloud */
  PointCloud          m_accumulated_cloud; 
  /** \brief ROS Node Handler for the \ref PointCloudAccumulator */
  ros::NodeHandle     m_node_handler; 
  /** \brief ROS Publisher for the completed accumulated PointCloud */
  ros::Publisher      m_accumulated_point_cloud_publisher; 
};

/**
 * \class MyPointRepresentation
 * \brief A new representation of PointClouds to include {X,Y,Z,Curvature} information.
 */
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:

  /**
   * \brief Standard constructor.
   * \details Set the number of dimensions for this PCL::Point representation to 4.
   */
  MyPointRepresentation ()
  {
    nr_dimensions_ = 4;
  }

  /**
   * \brief Override the copyToFloatArray method so that we can define the feature vector for this
   * kind of point cloud.
   *
   * @param point_in Input PCL::Point which includes normal information.
   * @param out floating point representation of {x,y,z} and the curvature of the Point.
   */
  virtual void copyToFloatArray (const PointNormalT &point_in, float * out) const
  {
    out[0] = point_in.x;
    out[1] = point_in.y;
    out[2] = point_in.z;
    out[3] = point_in.curvature;
  }
};

#endif /* POINTCLOUDACCUMULATOR_H_ */
