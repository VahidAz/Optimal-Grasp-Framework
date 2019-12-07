#include "pcl_interface.hpp"

#include <time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/random_sample.h>

#include "utilities.hpp"
#include "debug.hpp"


#ifdef DEBUGPI
  #define DBGPI(STMT) DBG(STMT)
#else
  #define DBGPI(STMT) 0
#endif


void PCLInterface::loadPointCloud(const char* const _path, const PointCloudPtr& _cloud)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(std::string(_path), *_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file\n");
    exit(-1);
  }

  DBGPI("Loaded "
        << _cloud->width * _cloud->height
        << " data points from " << _path << " with the following fields: "
        << std::endl);
}


void PCLInterface::loadPointCloud(const char* const _path, const PointNormalCloudPtr& _cloud)
{
  if (pcl::io::loadPCDFile<pcl::PointNormal>(std::string(_path), *_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file\n");
    exit(-1);
  }

  DBGPI("Loaded "
        << _cloud->width * _cloud->height
        << " data points from " << _path << " with the following fields: "
        << std::endl);
}


void PCLInterface::addToVis(const VisSPtr& _viewer, const PointCloudCPtr& _cloud,
                            const std::string& _name, const NormalCloudCPtr& _normals,
                            const PointCloudColor* const _color,
                            const unsigned& _point_size,
                            const unsigned& _normal_level,
                            const float& _normal_scale)
{
  if (_color != nullptr)
  {
    _viewer->addPointCloud<pcl::PointXYZ>(_cloud, *_color, _name);
  }
  else
  {
    _viewer->addPointCloud<pcl::PointXYZ>(_cloud, _name);
  }

  _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                            _point_size, _name);

  if (_normals != nullptr)
  {
    _viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(_cloud, _normals,
                                                              _normal_level, _normal_scale,
                                                              _name + "_Normals");
  }
}


void PCLInterface::visualize(const VisSPtr& _viewer)
{
  while (!_viewer->wasStopped())
  {
    _viewer->spin();
  }
}


void PCLInterface::calcNormals(const PointCloudCPtr& _cloud, const NormalCloudPtr& _normals,
                               const pcl::PointXYZ* _pView)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(_cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);

  // set view point, default is (0, 0, 0)
  if (_pView != nullptr)
  {
    ne.setViewPoint(_pView->x, _pView->y, _pView->z);
  }

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.03);

  // Compute the features
  ne.compute(*_normals);

  DBGPI("Normal Size: " << _normals->points.size() << std::endl);
}


void PCLInterface::sampleCloud(const PointCloudCPtr& _cloud,
                               const PointCloudPtr& _sampled_cloud,
                               unsigned _samples_num)
{
  if (_cloud->points.size() == _samples_num)
  {
    *_sampled_cloud.get() = *_cloud.get();

    DBGPI("Sampled point cloud size = Original point size: " << _sampled_cloud->size() << std::endl);

    return;
  }

  pcl::RandomSample<pcl::PointXYZ> random_sample;
  random_sample.setInputCloud(_cloud);

  if (_samples_num == 0)
  {
    _samples_num = _cloud->points.size();
  }

  random_sample.setSample(_samples_num);
  random_sample.setSeed(time(NULL));
  random_sample.filter(*_sampled_cloud);

  DBGPI("Sampled point cloud size: " << _sampled_cloud->size() << std::endl);
}


void PCLInterface::pruneInvalidContactPoints(const PointCloudCPtr& _cloud,
                                             const PointCloudCPtr& _sampled_cloud,
                                             const PointCloudPtr& _pruned_cloud,
                                             const NormalCloudCPtr& _cloud_normals,
                                             const NormalCloudPtr& _pruned_cloud_normals,
                                             const float& _finger_radius,
                                             const float& _pos_variance_threshold,
                                             const float& _normal_variance_threshold)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(_cloud);

  std::vector<int> point_id_radius_search;
  std::vector<float> point_radius_squared_distance;

  // Only for debugging
  DBGVIS
  (
    PointCloudPtr searchPointVis(new PointCloud);
    PointCloudPtr neighboursPointVis(new PointCloud);
  );

  for (const auto& search_point : _sampled_cloud->points)
  {
    DBGVIS
    (
      searchPointVis->clear();
      neighboursPointVis->clear();

      searchPointVis->points.push_back(search_point);
    );

    point_id_radius_search.clear();
    point_radius_squared_distance.clear();

    DBGPI("\nNeighbors within radius search at (" << search_point.x
          << " " << search_point.y
          << " " << search_point.z
          << ") with radius=" << _finger_radius << std::endl);

    if (kdtree.radiusSearch(search_point, _finger_radius, point_id_radius_search, point_radius_squared_distance) > 0)
    {
      for (size_t i = 0; i < point_id_radius_search.size(); ++i)
      {
        DBGPI(" " << _cloud->points[ point_id_radius_search[i] ].x
              << " " << _cloud->points[ point_id_radius_search[i] ].y
              << " " << _cloud->points[ point_id_radius_search[i] ].z
              << " (squared distance: " << point_radius_squared_distance[i] << ")" << std::endl);

        DBGVIS
        (
          if (i != 0)
          {
            neighboursPointVis->points.push_back(_cloud->points[ point_id_radius_search[i] ]);
          }
        );
      }

      pcl::Normal normal_search_point = _cloud_normals->at( point_id_radius_search[0] );

      DBGPI("Normal search point: " << normal_search_point.normal[0] << " "
            << normal_search_point.normal[1] << " "
            << normal_search_point.normal[2] << std::endl);

      float normal_dist_squared_sum = 0,
            pos_dist_squared_sum = 0;

      float tmpDistNorm;

      for (size_t i = 1; i < point_id_radius_search.size(); ++i)
      {
        pcl::Normal curr_normal = _cloud_normals->at( point_id_radius_search[i] );

        tmpDistNorm = 1 - Utilities::dotProduct(normal_search_point.normal[0],
                                                normal_search_point.normal[1],
                                                normal_search_point.normal[2],
                                                curr_normal.normal[0],
                                                curr_normal.normal[1],
                                                curr_normal.normal[2]);

        DBGPI("Normal[" << i << "]: " << curr_normal.normal[0] << " "
              << curr_normal.normal[1] << " "
              << curr_normal.normal[2] << " (distance: " << tmpDistNorm << ")" << std::endl);

        normal_dist_squared_sum += pow(tmpDistNorm, 2.0);

        pos_dist_squared_sum += point_radius_squared_distance[i];
      }

      float variance_pos = pos_dist_squared_sum / (point_radius_squared_distance.size() - 1);

      float variance_normal = normal_dist_squared_sum / (point_radius_squared_distance.size() - 1);

      DBGPI("Variance Pos: " << variance_pos << std::endl);
      DBGPI("Variance Normal: " << variance_normal << std::endl);

      if (variance_pos < _pos_variance_threshold && variance_normal < _normal_variance_threshold)
      {
         DBGPI("\nThis point is okay for considering\n");

        _pruned_cloud->push_back(search_point);
        _pruned_cloud_normals->push_back(normal_search_point);
      }
      else
      {
        DBGPI("\nThis point is rejected\n");
      }

      DBGPI("\n search point size: " << searchPointVis->size() << std::endl);
      DBGPI("\n neigh size: " << neighboursPointVis->size() << std::endl);

//      DBGVIS
//      (
//        VisSPtr viewer_search_point_neighbours(new pcl::visualization::PCLVisualizer("Current Search Point & Neighbours"));
//        viewer_search_point_neighbours->setBackgroundColor(0, 0, 0);
//        viewer_search_point_neighbours->addCoordinateSystem(0.1);
//        viewer_search_point_neighbours->initCameraParameters();
//        viewer_search_point_neighbours->registerKeyboardCallback(PCLInterface::pclKeyboardEventOccurred,
//                                                                 (void*)&viewer_search_point_neighbours);

//        PCLInterface::addToVis(viewer_search_point_neighbours, _cloud, "cloud", _cloud_normals);

//        PointCloudColor rgb_search_point(searchPointVis, 255, 0, 0);
//        PCLInterface::addToVis(viewer_search_point_neighbours, searchPointVis, "Search_Point",
//                               nullptr, &rgb_search_point, 9);

//        PointCloudColor rgb_neighbour_points(neighboursPointVis, 0, 0, 255);
//        PCLInterface::addToVis(viewer_search_point_neighbours, neighboursPointVis, "Neighbour_Search_Point",
//                               nullptr, &rgb_neighbour_points, 15);

//        PCLInterface::visualize(viewer_search_point_neighbours);

//        int aa;
//        std::cout << "Enter a digit for continuing ...\n";
//        std::cin >> aa;

//        viewer_search_point_neighbours->close();
//      );
    }
  }

  DBGPI("Pruned sampled point cloud size: " << _pruned_cloud->size() << std::endl);
}


void PCLInterface::pclKeyboardEventOccurred(const pcl::visualization::KeyboardEvent& _event, void* _viewer_void)
{
  VisSPtr viewer = *static_cast<VisSPtr*>(_viewer_void);

  if (_event.getKeySym() == "q" && _event.keyDown())
  {
    viewer->close();
  }
}


void PCLInterface::findingMaxDistCOM(const PointCloudCPtr& _cloud, const pcl::PointXYZ& _COM, float& _max_dist)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(_cloud);

  kdtree.setSortedResults(true);

  std::vector<int> point_id_radius_search;
  std::vector<float> point_radius_squared_distance;

  if (kdtree.radiusSearch(_COM, 30, point_id_radius_search, point_radius_squared_distance) > 0)
  {
    _max_dist = point_radius_squared_distance.back();
  }
}
