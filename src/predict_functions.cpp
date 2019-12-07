#include "predict_functions.hpp"

#include "utilities.hpp"
#include "debug.hpp"
#include "parallel_jaw_constrains.hpp"


#ifdef DEBUGPF
  #define DBGPF(STMT) DBG(STMT)
#else
  #define DBGPF(STMT) 0
#endif


std::map<ContactPointsSet, unsigned> PredictFunctions::graspValidityLookUpTable_;
PointCloudCPtr PredictFunctions::pruned_sampled_cloud_ = nullptr;
NormalCloudCPtr PredictFunctions::pruned_sampled_cloud_normals_ = nullptr;


bool PredictFunctions::isValidState(const SearchStateSPtr& _curr_search_state_ptr,
                                    const END_EFFECTOR_T& _type)
{
  if (_type == parallel_jaw) // or 0
  {
    return parallel_jaw_validity_check(_curr_search_state_ptr);
  }
}


bool PredictFunctions::parallel_jaw_validity_check(const SearchStateSPtr& _curr_search_state_ptr)
{
  DBGPF("\nCurrent State in Predict Function:\n");
  DBGPF(*_curr_search_state_ptr.get());

  ContactPointSet_Set enums_res, // Enum result
                      enums_in;

  // Adding contact point sets in this state to input for enumoration calculation
  enums_in.reserve(_curr_search_state_ptr->fingers_super_contacts_.size());
  for (const auto& cp : _curr_search_state_ptr->fingers_super_contacts_)
  {
    enums_in.push_back(cp.cPsSet_);
  }

  // Make all enumorations
  std::vector<unsigned> enumSoFar;
  Utilities::enumVecs(enums_in, 0, enumSoFar, enums_res);

  DBGPF("\n<Enumoration Result>\n");
  DBGPF("The number of grasp set after enumoration: " << enums_res.size() << std::endl);
  for (const auto& en : enums_res)
  {
    for (const auto& item : en)
    {
      DBGPF(item << "\t");
    }
    DBGPF(std::endl);
  }

  // Check whether there is any valid grasp in G(S) of this state
  // If there is not any valid grasp in this state, this state will not be expanded

  // Before above check we can have two other checks like Force Closure and low quality value
  _curr_search_state_ptr->updateForceClosureStatus();

  // We don't need to have anything regarding state ID since we don't check for Root state

//  if (_curr_search_state_ptr->force_closure_ == 1) // Not Force Closure
//  {
//    DBGPF("\nPrune this state, it is not a Force Closure State");

//    return false;
//  }

//  if (_curr_search_state_ptr->quality_ == NEG_INFINITY) // NEG_INFINITY
//  {
//    DBGPF("\nPrune this state, it is a State with NEG_INFINITY quality\n");

//    return false;
//  }

  unsigned res = 0;
  unsigned countValidGrasps = 0;

  for (const auto& en : enums_res)
  {
    DBGPF("\n<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>\n");

    res = 0;

    for (const auto& item : en)
    {
      DBGPF(item << "\t");
    }
    DBGPF(std::endl);

    // If the grasp is *NOT* available in LookUp Table
    std::map<ContactPointsSet, unsigned>::iterator ite = graspValidityLookUpTable_.find(en);

    if (ite == graspValidityLookUpTable_.end())
    {
      DBGPF("\nNot available in LookUp Table\n");

      res = parallel_jaw_feasibility_check(en);

      DBGPF("\nRes: " << res);

      // 0, False
      // 1, True
      graspValidityLookUpTable_[en] = unsigned(res);

      // Add the reverse order of current enum to LookUp table
      unsigned first_index = en[0];
      unsigned sec_index = en[1];

      if (first_index == sec_index)
      {
        continue; // No need to add reverse of state with similar contact points
      }

      ContactPointsSet reverseState;
      reverseState.push_back(sec_index);
      reverseState.push_back(first_index);

      DBGPF("\nReverse order of this grasp\n");
      for (const auto& item : reverseState)
      {
        DBGPF(item << "\t");
      }
      DBGPF(std::endl);

      if (res)
      {
        graspValidityLookUpTable_[reverseState] = 2; // Reverse State and True
      }
      else
      {
        graspValidityLookUpTable_[reverseState] = 3; // Reverse State and False
      }
    }
    else
    {
      DBGPF("\nAvailable in LookUp Table\n");

      res = graspValidityLookUpTable_[en];

      DBGPF("\nRes: " << res);
    }

    // It is better to ask from author with how many valid grasp in each state we will proceed
    if (res == 1) // If there is only one feasible grasp we will expand this state
    {
      countValidGrasps++;
    }
  }

  DBGPF("\nCountValidGrasps: " << countValidGrasps << "\n");

  if (countValidGrasps >= 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool PredictFunctions::parallel_jaw_feasibility_check(const ContactPointsSet& _curr_contact_point_set)
{
  unsigned first_index = _curr_contact_point_set[0];
  unsigned sec_index = _curr_contact_point_set[1];

  if (first_index == sec_index)
  {
    DBGPF("\nThey have same index\n");

    return false;
  }

  DBGPF("\nFirst Index: " << first_index << std::endl);
  DBGPF("\nSec Index: " << sec_index << std::endl);

  pcl::Normal first_normal = pruned_sampled_cloud_normals_->at(first_index);
  pcl::Normal sec_normal   = pruned_sampled_cloud_normals_->at(sec_index);

  DBGPF("\nFirst Normal: " << first_normal.normal[0] << "\t"
        << first_normal.normal[1] << "\t" << first_normal.normal[2] << std::endl);
  DBGPF("\nSecond Normal: " << sec_normal.normal[0] << "\t"
        << sec_normal.normal[1] << "\t" << sec_normal.normal[2] << std::endl);

  pcl::PointXYZ first_point = pruned_sampled_cloud_->points[first_index];
  pcl::PointXYZ sec_point = pruned_sampled_cloud_->points[sec_index];

  DBGPF("\nFirst Point: " << first_point.x << "\t" << first_point.y << "\t"
        << first_point.z << std::endl);
  DBGPF("\n Second Point: " << sec_point.x << "\t" << sec_point.y << "\t"
        << sec_point.z << std::endl);

  float normDist = Utilities::dotProduct(first_normal.normal[0],
                                         first_normal.normal[1],
                                         first_normal.normal[2],
                                         sec_normal.normal[0],
                                         sec_normal.normal[1],
                                         sec_normal.normal[2]);

  DBGPF("\nDot product result: " << normDist << std::endl);

  // Cheking Both norm and dist constrains
  if (normDist == -1)//-0.8 && normDist >= -1) // In front of each other
  {
    DBGPF("These two points are in front of each other");

    float distDiff = Utilities::euclDist3D(first_point.x, first_point.y, first_point.z,
                                           sec_point.x, sec_point.y, sec_point.z);

    DBGPF("\nPosition dist: " << distDiff << std::endl);

    if (distDiff >= PARALLEL_JAW_MIN_DIST && distDiff <= PARALLEL_JAW_MAX_DIST)
    {
      DBGPF("\nPass Dist Constraint\n");

      return true;
    }

    ///*** TODO: Calculate finger length to make sure we can put fingers at these points

  }

  return false;
}


void PredictFunctions::setCloudNormals(const PointCloudCPtr& _pruned_sampled_cloud,
                                       const NormalCloudCPtr& _pruned_sampled_cloud_normals)
{
  assert(_pruned_sampled_cloud->points.size() > 0);
  assert(_pruned_sampled_cloud_normals->points.size() > 0);

  pruned_sampled_cloud_ = _pruned_sampled_cloud;
  pruned_sampled_cloud_normals_ = _pruned_sampled_cloud_normals;
}
