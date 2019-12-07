#include "search_state_successor_generators.hpp"

#include <unordered_set>
#include <algorithm>
#include <random>
#include <array>

#include "dkm.hpp"

#include "search_state.hpp"
#include "utilities.hpp"
#include "search_state_quality_functions.hpp"
#include "common_defs.hpp"
#include "pcl_interface.hpp"
#include "debug.hpp"


#ifdef DEBUGSSSG
  #define DBGSSSG(STMT) DBG(STMT)
#else
  #define DBGSSSG(STMT) 0
#endif


PointCloudCPtr SearchStateSuccessorsGenerators::pruned_sampled_cloud_ = nullptr;
NormalCloudCPtr SearchStateSuccessorsGenerators::pruned_sampled_cloud_normals_ = nullptr;


bool SearchStateSuccessorsGenerators::randomSuccessorGenerator(const SearchStateSPtr& _curr_state_ptr,
                                                               const unsigned& _branch_factor,
                                                               unsigned& _id_counter,
                                                               SearchStateSPtrSet* _successors)
{
  DBGSSSG("\n^^^^^^^^^^ Random Successor Generator ^^^^^^^^^^\n");
  DBGSSSG(*_curr_state_ptr << std::endl);

  SuperContactSet_Set resSCSSet;

  for (const auto& curr_super_contact : _curr_state_ptr->fingers_super_contacts_)
  {
    if (curr_super_contact.cPsSet_.size() == 1)
    {
      SuperContactSet tmpSCSet;
      tmpSCSet.push_back(curr_super_contact);

      resSCSSet.push_back(tmpSCSet);

      continue;
    }

    SuperContactSet currSCSet;

    unsigned divide_index = curr_super_contact.cPsSet_.size() / _branch_factor;

    if (divide_index == 0)
    {
      divide_index = 1;
    }
//    unsigned divide_mod = curr_super_contact.cPsSet_.size() % _branch_factor;
    DBGSSSG("\nDivide Index: " << divide_index << std::endl);

    unsigned lower_level = 0;
    unsigned upper_level = divide_index;// + divide_mod;

    // Dividing randomly
    ContactPointsSet randNums;
    randNums.resize(curr_super_contact.cPsSet_.size());
    randNums = curr_super_contact.cPsSet_;

    auto rng = std::default_random_engine{};
    std::shuffle(std::begin(randNums), std::end(randNums), rng);

    unsigned countStates = 0;
    while (upper_level <= curr_super_contact.cPsSet_.size())
    {
      if (countStates == _branch_factor - 1)
      {
        upper_level = curr_super_contact.cPsSet_.size();
      }

      SuperContact tmpSuperContact;
      copy(randNums.begin() + lower_level,
           randNums.begin() + upper_level,
           std::back_inserter(tmpSuperContact.cPsSet_));

      currSCSet.push_back(tmpSuperContact);

      lower_level += divide_index;
      upper_level += divide_index;

      countStates++;
    }
    resSCSSet.push_back(currSCSet);

//    // Dividing sequentially
//    while (upper_level <= curr_super_contact.cPsSet_.size())
//    {
//      SuperContact tmpSuperContact;
//      copy(curr_super_contact.cPsSet_.begin() + lower_level,
//           curr_super_contact.cPsSet_.begin() + upper_level,
//           std::back_inserter(tmpSuperContact.cPsSet_));

//      currSCSet.push_back(tmpSuperContact);

//      lower_level += upper_level;
//      upper_level += divide_index;
//    }
//    resSCSSet.push_back(currSCSet);
  }

  DBGSSSG("Generated super contact sets size: " << resSCSSet.size() << std::endl);

  for (const auto& curr_SCSSet : resSCSSet)
  {
    DBGSSSG("Super contact set size: " << curr_SCSSet.size() << std::endl);

    for (const auto& curr_sc : curr_SCSSet)
    {
      DBGSSSG(curr_sc << std::endl);
    }
  }

  return generatingSuccessors(_curr_state_ptr, resSCSSet, _id_counter, _successors);
}


bool SearchStateSuccessorsGenerators::repeatedSearchState(const SearchState& _state,
                                                          SearchStateSPtrSet* _successors)
{
  // All successors are in a same level, so we don't need compare their parents
  for (const auto& _curr_successor : *_successors)
  {
    if (_state == *_curr_successor.get())
    {
      return true;
    }
  }

  return false;
}


bool SearchStateSuccessorsGenerators::kmeanSuccessorGenerator(const SearchStateSPtr& _curr_state_ptr,
                                                              const unsigned& _branch_factor,
                                                              unsigned& _id_counter,
                                                              SearchStateSPtrSet* _successors)
{
  DBGSSSG("\n\n KMEAN SUCCESSOR GENERATOR \n\n");

  SuperContactSet_Set resSCSSet;

  for (const auto& curr_super_contact : _curr_state_ptr->fingers_super_contacts_)
  {
    std::vector<std::array<float, 6>> data;

    SuperContactSet superCS;

    for (const auto& curr_contact_point : curr_super_contact.cPsSet_)
    {
      std::array<float, 6> tmpData = { pruned_sampled_cloud_->points[ curr_contact_point ].x,
                                       pruned_sampled_cloud_->points[ curr_contact_point ].y,
                                       pruned_sampled_cloud_->points[ curr_contact_point ].z,
                                       pruned_sampled_cloud_normals_->at( curr_contact_point ).normal[0],
                                       pruned_sampled_cloud_normals_->at( curr_contact_point ).normal[1],
                                       pruned_sampled_cloud_normals_->at( curr_contact_point ).normal[2]
                                     };

      data.push_back(tmpData);
    }

    // Printing data out
    DBGSSSG("\nData Input for KMean\n");
    for (const auto& d : data)
    {
      for (const auto& m : d)
      {
        DBGSSSG(m << "\t");
      }
      DBGSSSG("\n");
    }

    // Applying KMean
    auto means_clusters = dkm::kmeans_lloyd(data, _branch_factor);

    auto means = std::get<0>(means_clusters); // Mean for each cluster
    auto clusters = std::get<1>(means_clusters); // Cluster number for each data

    // Verify results
    assert(clusters.size() == data.size());

    // Printing Mean for each cluster
    DBGSSSG("\nMean for each cluster: \n");
    for (const auto& mean : means)
    {
      for (const auto& val : mean)
      {
        DBGSSSG(val << "\t");
      }
      DBGSSSG("\n");
    }

    // Printing cluster number for each data
    DBGSSSG("\nCluster Number for each data: \n");
    for (const auto& res : clusters)
    {
      DBGSSSG(res << std::endl);
    }

    // Making new state from each cluster
    PointCloudPtr currClusterPointCloud(new PointCloud);

    DBGSSSG("\nMaking new state from each cluster \n");
    for (size_t bf = 0; bf < _branch_factor; ++bf)
    {
      DBGSSSG("Current cluster number: " << bf << std::endl);

      SuperContact newSC;
      currClusterPointCloud->points.clear();

      for (size_t i = 0; i < clusters.size(); ++i)
      {
        if (bf == clusters[i])
        {
          newSC.cPsSet_.push_back(i);

          DBGVISSG
          (
            currClusterPointCloud->points.push_back(pruned_sampled_cloud_->points[i]);
          );
        }
      }

      DBGSSSG("New state for current cluster: " << std::endl);
      DBGSSSG(newSC << std::endl);

      superCS.push_back(newSC);

      DBGVISSG
      (
        // Making viewers
        VisSPtr viewer_cluster_point(new pcl::visualization::PCLVisualizer("Current_Cluster"));
        viewer_cluster_point->setBackgroundColor(0, 0, 0);
        viewer_cluster_point->addCoordinateSystem(0.1);
        viewer_cluster_point->initCameraParameters();
        viewer_cluster_point->registerKeyboardCallback(PCLInterface::pclKeyboardEventOccurred, (void*)&viewer_cluster_point);

        PCLInterface::addToVis(viewer_cluster_point, currClusterPointCloud, "cloud");
        PCLInterface::visualize(viewer_cluster_point);
      );
    }
    resSCSSet.push_back(superCS);
  }

  return generatingSuccessors(_curr_state_ptr, resSCSSet, _id_counter, _successors);
}


bool SearchStateSuccessorsGenerators::generatingSuccessors(const SearchStateSPtr& _curr_state_ptr,
                                                           const SuperContactSet_Set& _sCSS,
                                                           unsigned& _id_counter,
                                                           SearchStateSPtrSet* _successors)
{
  //** This part should be asked from author, we make states out of cartesian product of states

  SuperContactSet_Set res_cart;
  Utilities::cart_product(_sCSS, res_cart);
  DBGSSSG("Cart product res size: " << res_cart.size() << std::endl);

  std::vector<float> qualitiesVec;

  for (auto& tmpState : res_cart)
  {
    SearchState new_state;

    for (size_t ind = 0; ind < new_state.fingers_super_contacts_.size(); ++ind)
    {
      SuperContact tmpSC = tmpState[ind];

      (new_state.fingers_super_contacts_[ind]).cPsSet_.resize(tmpSC.cPsSet_.size());
      (new_state.fingers_super_contacts_[ind]).cPsSet_ = tmpSC.cPsSet_;
    }

    //** This part should be asked as well since for expanding we need

    float new_state_quality = SearchStateQualityFunctions::calcQualityByQ1(new_state);
    DBGSSSG("new_state_quality: " << new_state_quality << std::endl);

/***** We first expand a state to obtain the children states without checking the validity of any child

    // All of these pruning could happen in AStar inside Predict function
    // These states couldn't be a valid state in reachability manifold
    if (!new_state.isValid()) // Ignore Single Element State with Same Contact Point
    {
      DBGSSSG("\n\n\t\t&&&&&&&&&& Similar Single Element State &&&&&&&&&&\n\n");

      continue;
    }

    // This part could be removed once we convert states to feature states
    // Since we don't have similar state in feature space
    if (repeatedSearchState(new_state, _successors)) // Igonre Repeated States
    {
      DBGSSSG("\n\n\t\t%%%%%%%%%% Repeated State, ignore it %%%%%%%%%%\n\n");

      continue;
    }

    // Ignore Invalid States Like States with Coplanar Contact Points
    // Which there is no way to have a convex hull around them
    // Here we can ignore non force closure grasps (<= 0)
    if (new_state_quality == NEG_INFINITY)
    {
      DBGSSSG("\n\n\t\t########## NEG_INFINITY ##########\n\n");

      continue;
    }

    if (new_state_quality <= 0)
    {
      DBGSSSG("\n\n\t\t########## Non Force Closure State ##########\n\n");

      continue;
    }

*****/

    if (fabs(_curr_state_ptr->quality_ - new_state_quality) < THRESHOLD_IGNORE_DIFF_QUALITY)
    {
      new_state_quality = _curr_state_ptr->quality_;
    }

    new_state.quality_ = new_state_quality;

    new_state.g_ = _curr_state_ptr->quality_ - new_state_quality;

    new_state.id_ = _id_counter;
    _id_counter++;

    new_state.parent_ = _curr_state_ptr;

    qualitiesVec.push_back(new_state_quality);

    std::shared_ptr<SearchState> tmpSSPtr(new SearchState(new_state));
    _successors->push_back(tmpSSPtr);
  }

  if (_successors->size() == 0)
  {
    return false;
  }

  auto max_qual = *std::max_element(std::begin(qualitiesVec), std::end(qualitiesVec));
  DBGSSSG("Max quality: " << max_qual << std::endl);

  _curr_state_ptr->h_ = _curr_state_ptr->quality_ - max_qual;

  DBGSSSG("Successors size: " << _successors->size() << std::endl);
  for(const auto& tmpS : *_successors)
  {
    DBGSSSG(*tmpS << std::endl);
  }

  DBGSSSG("\n Update Current State with h value\n");
  DBGSSSG(*_curr_state_ptr << std::endl);

  return true;
}


void SearchStateSuccessorsGenerators::setCloudNormals(const PointCloudCPtr& _pruned_sampled_cloud,
                                                      const NormalCloudCPtr& _pruned_sampled_cloud_normals)
{
  assert(_pruned_sampled_cloud->points.size() > 0);
  assert(_pruned_sampled_cloud_normals->points.size() > 0);

  pruned_sampled_cloud_ = _pruned_sampled_cloud;
  pruned_sampled_cloud_normals_ = _pruned_sampled_cloud_normals;
}
