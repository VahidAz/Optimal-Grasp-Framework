#include "search_state_quality_functions.hpp"

#include <unordered_set>

#include "utilities.hpp"
#include "pcl_interface.hpp"
#include "common_defs.hpp"
#include "debug.hpp"


#ifdef DEBUGSSQF
  #define DBGSSQF(STMT) DBG(STMT)
#else
  #define DBGSSQF(STMT) 0
#endif


PointCloudCPtr SearchStateQualityFunctions::pruned_sampled_cloud_ = nullptr;
NormalCloudCPtr SearchStateQualityFunctions::pruned_sampled_cloud_normals_ = nullptr;
Eigen::Vector3f SearchStateQualityFunctions::centroid_ = Eigen::Vector3f(0,0,0);
float SearchStateQualityFunctions::fric_coeff_ = 0.0;
float SearchStateQualityFunctions::max_dist_COM_ = -1.0;
float SearchStateQualityFunctions::wrench_h_w_ = 1.0;


void SearchStateQualityFunctions::setCloudNormals(const PointCloudCPtr& _pruned_sampled_cloud,
                                                  const NormalCloudCPtr& _pruned_sampled_cloud_normals,
                                                  const pcl::PointXYZ& _centroid_point,
                                                  const float& _fric_coeff,
                                                  const float& _max_dist_COM,
                                                  const float& _wrench_h_w)
{
  assert(_pruned_sampled_cloud->points.size() > 0);
  assert(_pruned_sampled_cloud_normals->points.size() > 0);

  pruned_sampled_cloud_ = _pruned_sampled_cloud;
  pruned_sampled_cloud_normals_ = _pruned_sampled_cloud_normals;

  centroid_ << _centroid_point.x, _centroid_point.y, _centroid_point.z;

  assert(_fric_coeff >= 0);
  fric_coeff_ = _fric_coeff;

  assert(_max_dist_COM > 0);
  max_dist_COM_ = _max_dist_COM;

  assert(_wrench_h_w > 0);
  wrench_h_w_ = _wrench_h_w;
}


float SearchStateQualityFunctions::calcQualityByQ1(SearchState& _state)
{
  /*
   *  > 0     , forceClosure
   *  <= 0    , not forceClosure
   * -Infinity, Invalid state for expanding
   */

  DBGSSQF("\n--------------- Q1 Quality ---------------\n");
  DBGSSQF(_state << std::endl);

  float qualityVal = NEG_INFINITY;

  bool reduced_Dimension = false;

  unsigned dimension = 6, // Wrench dimension
           polyVecsNum = 8; // Number of vectors for estimating wrench cone as a polyhedral

  // Calculating Polyhedral Vertices
  std::vector<Eigen::Vector3f> polyVecs;
  SearchStateQualityFunctions::discreatizingFrictionCone(polyVecsNum, polyVecs);

  unsigned numCPsUnion = _state.numCPsWORepeat(); // Number of contact points without repeatation
  DBGSSQF("Number of contact points after union: " << numCPsUnion << std::endl);

  if (numCPsUnion <= 1)
  {
    return qualityVal;
  }

  if (numCPsUnion == 2)
  {
    reduced_Dimension = true;
    dimension = 3;
  }

  unsigned arrayWrenchSize = numCPsUnion * polyVecsNum * dimension;
  DBGSSQF("Array wrench size: " << arrayWrenchSize << std::endl);

  if (!reduced_Dimension)
  {
    double* wrenches_array = new double[arrayWrenchSize];

    computeWrenches(_state, wrenches_array, polyVecs, polyVecsNum);

    if (!makingQhull(_state, wrenches_array, arrayWrenchSize, dimension, qualityVal))
    {
      return NEG_INFINITY;
    }

    delete[] wrenches_array;
  }
  else
  {
    double* forces_array = new double[arrayWrenchSize];
//    double* torques_array = new double[arrayWrenchSize];

    computeWrenches(_state, forces_array, polyVecs, polyVecsNum, true, false);
//    computeWrenches(_state, torques_array, polyVecs, polyVecsNum, false, true);

    float qualityValForces = NEG_INFINITY;
//    float qualityValTorques = 0;

    makingQhull(_state, forces_array, arrayWrenchSize, dimension, qualityValForces);
//    makingQhull(_state, torques_array, arrayWrenchSize, dimension, qualityValTorques);

    delete[] forces_array;
//    delete[] torques_array;

    DBGSSQF("Forces convex quality: " << qualityValForces << std::endl);
//    DBGSSQF("Torques convex quality: " << qualityValTorques << std::endl);

//    qualityVal = (qualityValForces + qualityValTorques) / 2.0;

    qualityVal = qualityValForces;
  }

  return qualityVal;
}


void SearchStateQualityFunctions::discreatizingFrictionCone(const unsigned& _polyVecsNum,
                                                            std::vector<Eigen::Vector3f>& _polyVecs)
{
  float angle_step = 360 / (float)_polyVecsNum;

  float curr_angle;

  DBGSSQF("Linearized Circle:" << std::endl);

  for (size_t i = 0; i < _polyVecsNum; ++i)
  {
    curr_angle = angle_step * i;
    Eigen::Vector3f polyVec(cos(TORADIAN(curr_angle)), sin(TORADIAN(curr_angle)), 0);

    polyVec /= polyVec.norm();

    DBGSSQF(polyVec << "\n\n");

    _polyVecs.push_back(polyVec);
  }
}


void SearchStateQualityFunctions::computeWrenches(const SearchState& _state,
                                                  double* _wrenches_array,
                                                  const std::vector<Eigen::Vector3f>& _polyVecs,
                                                  const unsigned& _polyVecsNum,
                                                  const bool& _only_forces,
                                                  const bool& _only_torques)
{
  DBGSSQF("\n********* Wrench Computation **********\n");

  DBGSSQF("Fric_Coef: " << fric_coeff_ << std::endl);
  DBGSSQF("Max Dist: " << max_dist_COM_ << std::endl);
  DBGSSQF("Wrench_h_w: " << wrench_h_w_ << std::endl);

  // Only for visualization
  DBGVIS
  (
    PointCloudPtr forces_cloud(new PointCloud);
    PointCloudPtr torques_cloud(new PointCloud);
    NormalCloudPtr normals(new NormalCloud);
  );

  unsigned arrayCount = 0;

  std::unordered_set<unsigned> local_repeat_avoid;

  for (const auto& curr_super_contact : _state.fingers_super_contacts_)
  {
    for (const auto& curr_contact : curr_super_contact.cPsSet_)
    {
      // Its union of all points in super contacts in a state, so no duplicate
      if (local_repeat_avoid.find(curr_contact) != local_repeat_avoid.end())
      {
        continue;
      }

      local_repeat_avoid.insert(curr_contact);

      // Calculating transformation for local frame
      Eigen::Vector3f curr_point(pruned_sampled_cloud_->points[curr_contact].x,
                                 pruned_sampled_cloud_->points[curr_contact].y,
                                 pruned_sampled_cloud_->points[curr_contact].z);

      DBGSSQF("Current Point: " << curr_point << std::endl);

      pcl::Normal tmpNormal = pruned_sampled_cloud_normals_->at(curr_contact);

      Eigen::Vector3f curr_norm(tmpNormal.normal[0], tmpNormal.normal[1], tmpNormal.normal[2]);
      DBGSSQF("Current Point Normal: " << curr_norm << std::endl);

      Eigen::Vector3f preVec(-curr_norm[1], curr_norm[0], 0);
      if (preVec.norm() < 1e-8)
      {
        preVec = Eigen::Vector3f(-curr_norm[2], 0, curr_norm[0]);

        if (preVec.norm() < 1e-8)
        {
          preVec = Eigen::Vector3f(0, -curr_norm[2], curr_norm[1]);
        }
      }
      preVec = preVec/preVec.norm();
      DBGSSQF("preVec: " << preVec << std::endl);

      Eigen::Vector3f thirdVec = curr_norm.cross(preVec);
      thirdVec = thirdVec/thirdVec.norm();
      DBGSSQF("thirdVec: " << thirdVec << std::endl);

      // Calculating polyhedral vectors
      for (size_t i = 0; i < _polyVecsNum; ++i)
      {
        Eigen::Vector3f polyVec = _polyVecs[i];

        Eigen::Vector3f force = curr_norm + (preVec * fric_coeff_ * polyVec(0,0)) + (thirdVec * fric_coeff_* polyVec(1,0));
        DBGSSQF("\nForce: " << force << std::endl);

        DBGVIS
        (
          pcl::PointXYZ tmpF(force(0,0), force(1,0), force(2,0));
          forces_cloud->push_back(tmpF);
          normals->push_back(tmpNormal);
        );

        Eigen::Vector3f torque = ((curr_point.cross(force) + (curr_norm * fric_coeff_* 0))) / max_dist_COM_;
        torque *= wrench_h_w_;
        DBGSSQF("\nTorque: " << torque << std::endl);

        DBGVIS
        (
          pcl::PointXYZ tmpT(torque(0,0), torque(1,0), torque(2,0));
          torques_cloud->push_back(tmpT);
//        normals->push_back(tmpNormal);
        );

        if (_only_forces)
        {
          _wrenches_array[arrayCount] = force(0,0); arrayCount++;
          _wrenches_array[arrayCount] = force(1,0); arrayCount++;
          _wrenches_array[arrayCount] = force(2,0); arrayCount++;
        }

        if (_only_torques)
        {
          _wrenches_array[arrayCount] = torque(0,0); arrayCount++;
          _wrenches_array[arrayCount] = torque(1,0); arrayCount++;
          _wrenches_array[arrayCount] = torque(2,0); arrayCount++;
        }
      }

      DBGVIS
      (
        forces_cloud->push_back(pruned_sampled_cloud_->points[curr_contact]);
        torques_cloud->push_back(pruned_sampled_cloud_->points[curr_contact]);
        normals->push_back(tmpNormal);
      );

//      break;
    }
//    break;

  }

  // Visualizing Forces & Torques
  // Making viewer for forces
  //***** There is a seg fault sometimes *****//
//  DBGVIS
//  (
//    VisSPtr viewer_forces(new pcl::visualization::PCLVisualizer("Forces"));

//    viewer_forces->setBackgroundColor(0, 0, 0);
//    viewer_forces->addCoordinateSystem(0.1);
//    viewer_forces->initCameraParameters();
//    PCLInterface::addToVis(viewer_forces, forces_cloud, "Forces", normals, nullptr, 9, 1, 0.3);
//    PCLInterface::visualize(viewer_forces);

//    // Making viewer for torques
//    VisSPtr viewer_torques(new pcl::visualization::PCLVisualizer("Torques"));
//    viewer_torques->setBackgroundColor(0, 0, 0);
//    viewer_torques->addCoordinateSystem(0.1);
//    viewer_torques->initCameraParameters();
//    PCLInterface::addToVis(viewer_torques, torques_cloud, "Torque", normals, nullptr, 9, 1, 0.2);
//    PCLInterface::visualize(viewer_torques);
//  );

}


bool SearchStateQualityFunctions::makingQhull(SearchState& _state, double* const _wrenches_array,
                                              const unsigned& _arrayWrenchSize,
                                              const unsigned& _dimension,
                                              float& _quant)
{
  // Making Qhull
  DBGSSQF("\n ~~~~~~~~~~ Qhull init ~~~~~~~~~~\n");

  boolT ismalloc;
  int curlong, totlong, exitcode;
  char options[200];
  facetT *facet = nullptr;
  int i;

  ismalloc = False;   // True if qh_freeqhull should 'free(array)'
  FILE *qhfp = fopen("logfile-qhull", "w");
  if (!qhfp)
  {
    DBGSSQF("Could not open qhull logfile!\n");
    qh_init_A(NULL, stdout, stderr, 0, NULL);
  }
  else
  {
    qh_init_A(NULL, qhfp, qhfp, 0, NULL);
  }

  if ((exitcode = setjmp(qh errexit)))
  {
    fprintf(stderr, "QUALITY: 0 volume, quick exit, Fail to make convex hull!\n");
    qh NOerrexit = True;
    qh_freeqhull(!qh_ALL);
    qh_memfreeshort(&curlong, &totlong);
    if (curlong || totlong)
      fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",
              totlong, curlong);
    if (qhfp) { fclose(qhfp); }
    return false;
  }

  sprintf(options, "qhull Pp n Qx C-0.0001");
  qh_initflags(options);
  qh_init_B(&_wrenches_array[0], _arrayWrenchSize/_dimension, _dimension, ismalloc);
  qh_qhull();
  qh_check_output();
  qh_getarea(qh facet_list);
  if (qhfp) { fclose(qhfp); }

  float hullArea = qh totarea;
  float hullVolume = qh totvol;
  unsigned numFacets = qh num_facets;

  _quant = closestDistQhull2Origin(facet);

  // Non force closure grasps are ignored in higher level
  // because quant value is correct when the grasp is force closure
  if (_quant > 0)
  {
    _state.force_closure_ = 1;
  }
  else if (_quant <= 0)
  {
    _state.force_closure_ = 0;
  }

  qh NOerrexit = True;
  qh_freeqhull(!qh_ALL);
  qh_memfreeshort(&curlong, &totlong);
  if (curlong || totlong)
  {
    fprintf(stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",
            totlong, curlong);
  }

  DBGSSQF("Qhull SUCCESS\n"
          << "\n\nQhull_Area: " << hullArea
          << "\nQhull_Volume: " << hullVolume
          << "\nFacets Nums:  " << numFacets << std::endl);

  delete[] facet;

  return true;
}


bool SearchStateQualityFunctions::isForceClosure(const facetT* facet)
{
  // We recognize whether the grasp is force closure or not, also we can find minumum wrench here

  bool forceClosure = false;

  FORALLfacets
  {
    if (facet->offset > 0) // It means origin is outside of convex hull
    {
      DBGSSQF("QUALITY: NON FORCE CLOSURE GRASP" << std::endl);

      return forceClosure;
    }
  }

  DBGSSQF("FORCE CLOSURE GRASP" << std::endl);

  return !forceClosure;
}


float SearchStateQualityFunctions::closestDistQhull2Origin(facetT* facet)
{
  float minDist = std::numeric_limits<float>::max();

  FORALLfacets
  {
    if ((-facet->offset) < minDist)
    {
      minDist = (-facet->offset);
    }
  }

  return minDist; // Negative means it is not a force closure grasp
}
