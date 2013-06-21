/*
 * ObjectCandidateExtraction.h
 *
 *  Created on: Jun 20, 2013
 *      Author: badrobit
 */

#ifndef OBJECTCANDIDATEEXTRACTION_H_
#define OBJECTCANDIDATEEXTRACTION_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class ObjectCandidateExtraction
{
public:
	ObjectCandidateExtraction();
	virtual ~ObjectCandidateExtraction();
	bool ExtractCandidateObjects( pcl::PointCloud<pcl::PointXYZ> &input_point_cloud, pcl::PointCloud<pcl::PointXYZ> &object_candidates );
};

#endif /* OBJECTCANDIDATEEXTRACTION_H_ */
