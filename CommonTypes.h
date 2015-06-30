#ifndef __INCLUDE_COMMON_TYPES_H
#define __INCLUDE_COMMON_TYPES_H

#include <pcl/point_types.h>

	/**
	 *
	 *	A macro which works like python's pass keyword
	 *
	 */
	#define PASS 1


	typedef pcl::PointXYZRGBA PointRGBA;
	typedef pcl::PointXYZ PointXYZ;

	typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
	typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudRGBAPtr;
	typedef pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr PointCloudRGBAConstPtr;

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
	typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudXYZConstPtr;

	typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
	typedef pcl::PointCloud<pcl::Normal>::Ptr PointCloudNormalPtr;


#endif