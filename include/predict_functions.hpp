#ifndef __PREDICT_FUNCTIONS_HPP_
#define __PREDICT_FUNCTIONS_HPP_


#include <map>

#include "search_state.hpp"
#include "common_defs.hpp"
#include "pcl_defs.hpp"


class PredictFunctions
{
  public:
    PredictFunctions() = delete;
    ~PredictFunctions() = delete;

    PredictFunctions(const PredictFunctions&) = delete;
    PredictFunctions& operator =(const PredictFunctions&) = delete;

    static void setCloudNormals(const PointCloudCPtr& _pruned_sampled_cloud,
                                const NormalCloudCPtr& _pruned_sampled_cloud_normals);

    static bool isValidState(const SearchStateSPtr& _curr_search_state_ptr,
                             const END_EFFECTOR_T& _end_effector_type);

  private:
    static bool parallel_jaw_validity_check(const SearchStateSPtr& _curr_search_state_ptr);
    static bool parallel_jaw_feasibility_check(const ContactPointsSet& _curr_contact_point_set);

    static std::map<ContactPointsSet, unsigned> graspValidityLookUpTable_;

    static PointCloudCPtr pruned_sampled_cloud_;
    static NormalCloudCPtr pruned_sampled_cloud_normals_;
};


#endif // __PREDICT_FUNCTIONS_HPP_
