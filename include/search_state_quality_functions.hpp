#ifndef __SEARCH_STATE_QUALITY_FUNCTIONS_HPP_
#define __SEARCH_STATE_QUALITY_FUNCTIONS_HPP_


#include "search_state.hpp"
#include "pcl_defs.hpp"

extern "C"
{
  #include "qhull/qhull_a.h"
}


class SearchStateQualityFunctions
{
  public:
    SearchStateQualityFunctions() = delete;
    ~SearchStateQualityFunctions() = delete;

    SearchStateQualityFunctions(const SearchStateQualityFunctions&) = delete;
    SearchStateQualityFunctions& operator =(const SearchStateQualityFunctions&) = delete;

    static void setCloudNormals(const PointCloudCPtr& _pruned_sampled_cloud,
                                const NormalCloudCPtr& _pruned_sampled_cloud_normals,
                                const pcl::PointXYZ& _centroid_point,
                                const float& _fric_coeff,
                                const float& _max_dist_COM,
                                const float& _wrench_h_w);

    static float calcQualityByQ1(SearchState& _state);

  private:
    static void discreatizingFrictionCone(const unsigned& _polyVecNum,
                                          std::vector<Eigen::Vector3f>& _polyVec);

    static void computeWrenches(const SearchState& _state,
                                double* _wrenches_array,
                                const std::vector<Eigen::Vector3f>& _polyVecs,
                                const unsigned& _polyVecsNum,
                                const bool& _only_forces = true,
                                const bool& _only_torques = true);

    static bool makingQhull(SearchState& _state,
                            double* const _wrenches_array,
                            const unsigned& _arrayWrenchSize,
                            const unsigned& _dimension,
                            float& _quant);

    static bool isForceClosure(const facetT* facet);

    static float closestDistQhull2Origin(facetT* facet);

    static PointCloudCPtr pruned_sampled_cloud_;
    static NormalCloudCPtr pruned_sampled_cloud_normals_;

    static Eigen::Vector3f centroid_;

    static float fric_coeff_;
    static float max_dist_COM_;
    static float wrench_h_w_;
};


#endif // __SEARCH_STATE_QUALITY_FUNCTIONS_HPP_
