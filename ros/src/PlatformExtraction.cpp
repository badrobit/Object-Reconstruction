/**
 * PlatformExtraction.cpp
 *
 * Copyright (c) Jun 20, 2013, badrobot
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "PlatformExtraction.h"

PlatformExtraction::PlatformExtraction( ros::NodeHandle i_node_handle ): m_node_handler( i_node_handle )
{
	m_extract_platform_service = m_node_handler.advertiseService( "ExtractPlatform", &PlatformExtraction::ExtractPlatform, this );
}

PlatformExtraction::~PlatformExtraction()
{
}

bool
PlatformExtraction::ExtractPlatform( hbrs_object_reconstruction::ExtractPlatform::Request &req,
                                     hbrs_object_reconstruction::ExtractPlatform::Response & res )
{
	ros::Time start, finish;
	start = ros::Time::now();

	pcl::PointCloud<pcl::PointXYZ> input_point_cloud;
	pcl::PointCloud<pcl::PointXYZ> extracted_platform_cloud;

	pcl::fromROSMsg( req.i_point_cloud, input_point_cloud );

	ROS_DEBUG_STREAM( "Platform Extraction: Input Point Cloud Size: " << (int)input_point_cloud.size() );

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f ( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
	pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );

	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setOptimizeCoefficients( true );
	segmentation.setModelType( pcl::SACMODEL_PLANE );
	segmentation.setMethodType( pcl::SAC_RANSAC );
	segmentation.setDistanceThreshold( 0.01 );
	segmentation.setInputCloud( input_point_cloud.makeShared() );
	segmentation.segment( *inliers, *coefficients );

	if( inliers->indices.size() == 0 )
	{
		ROS_WARN( "Platform Extraction: Could not find a plannar model in the data" );
	}

	pcl::ExtractIndices< pcl::PointXYZ > extract;
	extract.setInputCloud( input_point_cloud.makeShared() );
	extract.setIndices( inliers );
	extract.setNegative( false );
	extract.filter( extracted_platform_cloud );

	extract.setNegative( true );
	extract.filter( *cloud_f );
	input_point_cloud = *cloud_f;

	finish = ros::Time::now();
	ROS_INFO("platform_extraction_time=%lf", (finish.toSec() - start.toSec() ));
	ROS_INFO( "Platform Extraction Complete." );
}