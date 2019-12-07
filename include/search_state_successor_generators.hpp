#ifndef __SEARCH_STATE_SUCCESSORS_GENERATORS_HPP_
#define __SEARCH_STATE_SUCCESSORS_GENERATORS_HPP_


#include "search_state.hpp"
#include "pcl_defs.hpp"


class SearchStateSuccessorsGenerators
{
  public:
    SearchStateSuccessorsGenerators() = delete;
    ~SearchStateSuccessorsGenerators() = delete;

    SearchStateSuccessorsGenerators(const SearchStateSuccessorsGenerators&) = delete;
    SearchStateSuccessorsGenerators& operator =(const SearchStateSuccessorsGenerators&) = delete;

    static bool randomSuccessorGenerator(const SearchStateSPtr& _curr_state,
                                         const unsigned& _branch_factor,
                                         unsigned& _id_counter,
                                         SearchStateSPtrSet* _successors);

    static bool kmeanSuccessorGenerator(const SearchStateSPtr& _curr_state,
                                        const unsigned& _branch_factor,
                                        unsigned& _id_counter,
                                        SearchStateSPtrSet* _successors);

    static void setCloudNormals(const PointCloudCPtr& _pruned_sampled_cloud,
                                const NormalCloudCPtr& _pruned_sampled_cloud_normals);

  private:
    static bool repeatedSearchState(const SearchState& _state,
                                    SearchStateSPtrSet* _successors);

    static bool generatingSuccessors(const SearchStateSPtr& _curr_state_ptr,
                                     const SuperContactSet_Set& _sCSS,
                                     unsigned& _id_counter,
                                     SearchStateSPtrSet* _successors);

    static PointCloudCPtr pruned_sampled_cloud_;
    static NormalCloudCPtr pruned_sampled_cloud_normals_;
};


#endif // __SEARCH_STATE_SUCCESSORS_GENERATORS_HPP_
