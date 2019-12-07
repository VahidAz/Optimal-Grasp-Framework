#ifndef __ASTAR_H_
#define __ASTAR_H_


#include <unordered_map>

#include "search_state.hpp"
#include "common_defs.hpp"


class AStar
{
  public:
    AStar(const unsigned& _branch_factor = 2,
          const float& _heuristic_weight = 0,
          const unsigned& _successor_type = 0);

    void solve(const SearchState& _root,
               SearchStateCSPtr& _res,
               const END_EFFECTOR_T& _end_eff_type);

  private:
    unsigned id_counter_ = 0,
             branch_factor_,
             numNodesCreated_ = 0,
             numNodesVisited_ = 0,
             successor_type_ = 0; // 0 = random, 1 = kMean

    float heuristic_weight_ = 0;

    std::unordered_map<unsigned, SearchStateSPtrSet> idSuccessorsMap_;

    bool successorsGenerator(const SearchStateSPtr& _curr_state,
                             SearchStateSPtrSet* _successors = nullptr);

    inline bool isLeaf(const SearchState& _state)
    {
      return _state.singleElementSuperContacts();
    }

    void reconstructPath(const SearchState& _goal);
};


#endif // __ASTAR_H_
