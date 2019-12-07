#ifndef __PCL_INTERFACE_H_
#define __PCL_INTERFACE_H_


#include <pcl/visualization/cloud_viewer.h>

#include "pcl_defs.hpp"


typedef std::shared_ptr<pcl::visualization::PCLVisualizer> VisSPtr;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointCloudColor;


class PCLInterface
{
  public:
    PCLInterface() = delete;
    ~PCLInterface() = delete;

    PCLInterface(const PCLInterface&) = delete;
    PCLInterface& operator =(const PCLInterface&) = delete;

    static void loadPointCloud(const char* const _path, const PointCloudPtr& _cloud);
    static void loadPointCloud(const char* const _path, const PointNormalCloudPtr& _cloud);

    static void addToVis(const VisSPtr& _viewer, const PointCloudCPtr& _cloud,
                         const std::string& _name, const NormalCloudCPtr& _normals = nullptr,
                         const PointCloudColor* const _color = nullptr,
                         const unsigned& _point_size = 9,
                         const unsigned& _normal_level = 1,
                         const float& _normal_scale = 0.01);

    static void visualize(const VisSPtr& _viewer);

    static void calcNormals(const PointCloudCPtr& _cloud,
                            const NormalCloudPtr& _normals,
                            const pcl::PointXYZ* _pView = nullptr);

    static void sampleCloud(const PointCloudCPtr& _cloud,
                            const PointCloudPtr& _sampled_cloud,
                            unsigned _samples_num = 0);

    static void pruneInvalidContactPoints(const PointCloudCPtr& _cloud,
                                          const PointCloudCPtr& _sampled_cloud,
                                          const PointCloudPtr& _pruned_cloud,
                                          const NormalCloudCPtr& _cloud_normals,
                                          const NormalCloudPtr& _pruned_cloud_normals,
                                          const float& _finger_radius,
                                          const float& _pos_variance_threshold,
                                          const float& _normal_variance_threshold);

    static void pclKeyboardEventOccurred(const pcl::visualization::KeyboardEvent& _event, void* _viewer_void);

    static void findingMaxDistCOM(const PointCloudCPtr& _cloud, const pcl::PointXYZ& _COM, float& _max_dist);
};


#endif // __PCL_INTERFACE_H_
