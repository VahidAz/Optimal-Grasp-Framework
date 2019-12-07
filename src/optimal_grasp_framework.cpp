#include <time.h>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include "pcl_interface.hpp"
#include "search_state.hpp"
#include "search_state_quality_functions.hpp"
#include "search_state_successor_generators.hpp"
#include "predict_functions.hpp"
#include "a_star.hpp"
#include "common_defs.hpp"
#include "debug.hpp"


#ifdef DEBUGOGF
  #define DBGOGF(STMT) DBG(STMT)
#else
  #define DBGOGF(STMT) 0
#endif


int main(int argc, char** argv)
{
  //** TODO: Reading required parameters from text file or command prompt
  //** TODO: Applying overloaded move semantics and decreasing copy overhead


  clock_t startTimeOverall = clock(); // Overall start time


  // Whether input has normal vectors or not
  bool having_normals = true;


  DBGVIS
  (
    // Making viewers
    VisSPtr viewer_orig_samples(new pcl::visualization::PCLVisualizer("Orig Cloud & Normals & Samples"));
    viewer_orig_samples->setBackgroundColor(0, 0, 0);
    viewer_orig_samples->addCoordinateSystem(0.1);
    viewer_orig_samples->initCameraParameters();
    viewer_orig_samples->registerKeyboardCallback(PCLInterface::pclKeyboardEventOccurred, (void*)&viewer_orig_samples);

    VisSPtr viewer_pruned_sampled(new pcl::visualization::PCLVisualizer("Pruned Sampled Cloud & Normals"));
    viewer_pruned_sampled->setBackgroundColor(0, 0, 0);
    viewer_pruned_sampled->addCoordinateSystem(0.1);
    viewer_pruned_sampled->initCameraParameters();
    viewer_pruned_sampled->registerKeyboardCallback(PCLInterface::pclKeyboardEventOccurred, (void*)&viewer_pruned_sampled);
  );


  // Original Cloud and its Normals
  PointCloudPtr cloud_orig(new PointCloud);
  NormalCloudPtr cloud_normals(new NormalCloud);


  // Loading Point Cloud
  if (!having_normals) // Without Normal
  {
    PCLInterface::loadPointCloud(argv[1], cloud_orig);
  }
  else // With Normals. the normals are already inward, pcl_mesh_sampling_custom, pcl_1.8
  {
    PointNormalCloudPtr cloudN(new PointNormalCloud);
    PCLInterface::loadPointCloud(argv[1], cloudN);

    // Seperate cloud and its normals
    pcl::copyPointCloud(*cloudN, *cloud_orig);
    pcl::copyPointCloud(*cloudN, *cloud_normals);
  }


  //** TODO: Decreasing point cloud density


  clock_t startTimePCL = clock(); // Preprocessing PCL start time


  // Calculating Center of Mass, it could be given as an input parameter for each object
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_orig, centroid);
  DBGOGF("\nCentroid: " << centroid[0] << "\t" << centroid[1] << "\t" << centroid[2] << std::endl);
  pcl::PointXYZ centroid_point(centroid[0], centroid[1], centroid[2]);


  // Adding a Sphere in original centroid
  DBGVIS
  (
    viewer_orig_samples->addSphere(centroid_point, 0.007, 1.0, 0.0, 0.0, "centroid_sphere");
  );


  // Transforming COM of the object to the origin, now world frame and object frame are similar
  PointCloudPtr cloud(new PointCloud);
  Eigen::Affine3f transform_to_origin = Eigen::Affine3f::Identity();
  transform_to_origin.translation() << -centroid[0], -centroid[1], -centroid[2];
  DBGOGF("\nTransformation matrix for object to origin: " << std::endl);
  DBGOGF(transform_to_origin.matrix() << std::endl);
  pcl::transformPointCloud(*cloud_orig, *cloud, transform_to_origin);


  // Adding a Sphere in new centroid(Origin)
  pcl::PointXYZ origin_centroid(0, 0, 0);
  DBGVIS
  (
    viewer_orig_samples->addSphere(origin_centroid, 0.007, 1.0, 0.0, 1.0, "centroid_origin");
  );


  // Finding Max Distance from Centroid
  float maxDist = -1;
  PCLInterface::findingMaxDistCOM(cloud, origin_centroid, maxDist);
  DBGOGF("\nThe Maximum Distance: " << maxDist << std::endl);
  assert(maxDist > 0);


  // Estimating Normals if input doesn't have them


  //** TODO: I need to spend some time over this part for having something robust, because
  //**       Normals should point inward


  if (!having_normals)
  {
    PCLInterface::calcNormals(cloud, cloud_normals, &centroid_point);
  }


  // Sampling
  PointCloudPtr sampled_cloud(new PointCloud);
  PCLInterface::sampleCloud(cloud, sampled_cloud, cloud->points.size());
  DBGOGF("\nNumber of sampled points: " << sampled_cloud->size() << std::endl);


  // Finding Valid Samples
  PointCloudPtr pruned_sampled_cloud(new PointCloud);
  NormalCloudPtr pruned_sampled_cloud_normals(new NormalCloud);


  //** These parameters are important and maybe we have to change them object by object **//


  float finger_radius = 0.012,
        pos_var_threshold = 0.01,
        norm_var_threshold = 0.038;
  PCLInterface::pruneInvalidContactPoints(cloud, sampled_cloud,
                                          pruned_sampled_cloud,
                                          cloud_normals,
                                          pruned_sampled_cloud_normals,
                                          finger_radius, pos_var_threshold, norm_var_threshold);
  assert(pruned_sampled_cloud->points.size() > 0);
  assert(pruned_sampled_cloud_normals->points.size() > 0);
  DBGOGF("\nNumber of valid samples: " << pruned_sampled_cloud->size() << std::endl);


  clock_t endTimePCL = clock(); // PCL end time
  float pclTime = (float)(endTimePCL - startTimePCL) * 1000.0 / CLOCKS_PER_SEC;


  // Visualization
  DBGVIS
  (
    PCLInterface::addToVis(viewer_orig_samples, cloud, "cloud", cloud_normals);

    PointCloudColor rgb_sampled_cloud(sampled_cloud, 255, 0, 0);
    PCLInterface::addToVis(viewer_orig_samples, sampled_cloud, "sampled_cloud",
                           nullptr, &rgb_sampled_cloud, 9);

    PointCloudColor rgb_pruned_sampled_cloud(pruned_sampled_cloud, 0, 0, 255);
    PCLInterface::addToVis(viewer_orig_samples, pruned_sampled_cloud, "pruned_sampled_cloud",
                           nullptr, &rgb_pruned_sampled_cloud, 15);
    PCLInterface::addToVis(viewer_pruned_sampled, pruned_sampled_cloud, "pruned_sampled_cloud",
                           pruned_sampled_cloud_normals, &rgb_pruned_sampled_cloud, 6);

    PCLInterface::visualize(viewer_orig_samples);
    PCLInterface::visualize(viewer_pruned_sampled);
  );


  // End_Effector Type & Number of Fingers
  END_EFFECTOR_T end_eff_type = parallel_jaw ; // 0
  SearchState::setEndEffectorType(end_eff_type);


  // Creating root state
  SearchState root_state(0);
  for (auto& sc : root_state.fingers_super_contacts_)
  {
    sc.cPsSet_.resize(pruned_sampled_cloud->size());
    std::iota(sc.cPsSet_.begin(), sc.cPsSet_.end(), 0);
  }
  DBGOGF("\n<<< ROOT NODE >>>\n");
  DBGOGF(root_state << std::endl);


  // Set parameters for grasp quality
  float fric_coeff = 0.5774; // tan(theta) = fric_coeff
  float wrench_h_w = 1;


  SearchStateQualityFunctions::setCloudNormals(pruned_sampled_cloud,
                                               pruned_sampled_cloud_normals,
                                               centroid_point, fric_coeff,
                                               maxDist, wrench_h_w);


  // Set parameters for successor generator
  SearchStateSuccessorsGenerators::setCloudNormals(pruned_sampled_cloud, pruned_sampled_cloud_normals);


  // Set pointCloud and Normals for Predict Functions
  PredictFunctions::setCloudNormals(pruned_sampled_cloud, pruned_sampled_cloud_normals);


  unsigned branch_factor = 2; // partitioning factor
  SearchStateCSPtr aStarRes = nullptr; // AStar result


  clock_t startTimeAStar = clock(); // AStar start time


  // Running AStar
  AStar a_star(branch_factor);
  a_star.solve(root_state, aStarRes, end_eff_type);


  clock_t endTimeAStar = clock(); // AStar end time


  if (aStarRes != nullptr)
  {
    DBGOGF("\nGOAL STATE\n");
    DBGOGF(*aStarRes << std::endl);
  }
  else
  {
    DBGOGF("\nNO GOAL STATE, Astar Fails\n");
  }


  clock_t endTimeOverall = clock(); // Overall end time


  // Making pointcloud from result points for visualization
  PointCloudPtr graspPoints(new PointCloud);

  float X, Y, Z;
  unsigned count = 0;

  for (const auto& super_contact : aStarRes->fingers_super_contacts_)
  {
    for (const auto& contact : super_contact.cPsSet_)
    {
      graspPoints->points.push_back(pruned_sampled_cloud->points[contact]);

      count++;

      X += pruned_sampled_cloud->points[contact].x;
      Y += pruned_sampled_cloud->points[contact].y;
      Z += pruned_sampled_cloud->points[contact].z;
    }
  }

////  pcl::PointXYZ graspCenter(X, Y, Z);
//  DBGOGF("\nGrasp Center: " << X << "\t" << Y << "\t" << Z);

  float aStarTime = (float)(endTimeAStar - startTimeAStar) * 1000.0 / CLOCKS_PER_SEC;
  float overallTime = (float)(endTimeOverall - startTimeOverall) * 1000.0 / CLOCKS_PER_SEC;

  DBGOGF("\nPCL_Time:   " << pclTime     << "  ms\t" << pclTime/1000.0     << "  s\n"
         << "AStar_Time:   " << aStarTime   << "  ms\t" << aStarTime/1000.0   << "  s\n"
         << "Overall_Time: " << overallTime << "  ms\t" << overallTime/1000.0 << "  s\n");

  DBGVIS
  (
    // Making viewers
    VisSPtr viewer_result(new pcl::visualization::PCLVisualizer("Grasp Points"));
    viewer_result->setBackgroundColor(0, 0, 0);
    viewer_result->addCoordinateSystem(0.1);
    viewer_result->initCameraParameters();
    viewer_result->registerKeyboardCallback(PCLInterface::pclKeyboardEventOccurred, (void*)&viewer_result);

    PCLInterface::addToVis(viewer_result, cloud, "cloud", cloud_normals);

//    PointCloudColor rgb_sampled_cloud(sampled_cloud, 255, 0, 0);
    PCLInterface::addToVis(viewer_result, sampled_cloud, "sampled_cloud",
                           nullptr, &rgb_sampled_cloud, 12);

//    PointCloudColor rgb_pruned_sampled_cloud(pruned_sampled_cloud, 0, 0, 255);
    PCLInterface::addToVis(viewer_result, pruned_sampled_cloud, "pruned_sampled_cloud",
                           nullptr, &rgb_pruned_sampled_cloud, 9);

    PointCloudColor rgb_grasp_points(graspPoints, 0, 255, 255);
    PCLInterface::addToVis(viewer_result, graspPoints, "grasp_point_cloud",
                           nullptr, &rgb_grasp_points, 20);

    PCLInterface::visualize(viewer_result);
  );

  return 0;
}
