#ifndef __PCL_DEFS_H_
#define __PCL_DEFS_H_


#include <pcl/pcl_base.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudCPtr;

typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;
typedef NormalCloud::ConstPtr NormalCloudCPtr;

typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloud;
typedef PointNormalCloud::Ptr PointNormalCloudPtr;
typedef PointNormalCloud::ConstPtr PointNormalCloudCPtr;


#endif // __PCL_DEFS_H_
