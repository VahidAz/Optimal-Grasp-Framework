#include "a_star.hpp"

#include <set>

#include "a_star_functors.hpp"
#include "search_state_quality_functions.hpp"
#include "search_state_successor_generators.hpp"
#include "predict_functions.hpp"
#include "debug.hpp"


#ifdef DEBUGAS
  #define DBGAS(STMT) DBG(STMT)
#else
  #define DBGAS(STMT) 0
#endif


AStar::AStar(const unsigned& _branch_factor,
             const float& _heuristic_weight,
             const unsigned& _successor_type) :
             branch_factor_(_branch_factor),
             heuristic_weight_(_heuristic_weight),
             successor_type_(_successor_type)

{
  assert(_branch_factor >= MIN_VALID_BRANCH_FACTOR);
}


void AStar::solve(const SearchState& _root, SearchStateCSPtr& _res, const END_EFFECTOR_T& _end_eff_type)
{
  // OpenList
  std::set<IDSearchStateSPtrPair, CompareIDSearchStateSPtrPairByF> openList;

  // ClosedList
  std::unordered_map<unsigned, SearchStateSPtr> closedList;

  SearchStateSPtr root_pointer(new SearchState(_root));
  id_counter_++;
  numNodesCreated_++;

  root_pointer->quality_ = SearchStateQualityFunctions::calcQualityByQ1(*root_pointer.get());


  //** Root should be expanded anyway, so there is no need to check its validity **//

  /*
  // These two conditions could be removed, since root should be expanded anyway
  // Neg quality and non force closure
  if (root_pointer->quality_ == NEG_INFINITY)
  {
    DBGAS("\nRoot quality is negative\n");

    return;
  }

  if (root_pointer->force_closure_ != 1)
  {
    DBGAS("\nRoot is not force closure\n");

    return;
  }
  */

  if (!successorsGenerator(root_pointer)) // Look ahead for calculating h value
  {
    DBGAS("\nThere is NO successors for Root node\n");

    return;
  }

  root_pointer->f_ = root_pointer->g_ + (1 + heuristic_weight_) * root_pointer->h_;

  root_pointer->print();

  DBGAS("\nAfter updating h value of root\n");
  DBGAS(*root_pointer.get());

  openList.insert(make_pair(root_pointer->id_, root_pointer));

  SearchStateSPtrSet successors;

  bool visitGoal = false;

  while (!openList.empty())
  {
    DBGAS("\nOpenList Size: " << openList.size() << std::endl);

    IDSearchStateSPtrPair currentState = *openList.begin();

    numNodesVisited_++;

    openList.erase(openList.begin());


    // here we should enumorate all possible grasp and check the validity
    // and keep them in a look up table
    // We don't need to do it for root


    // Predict function should be called here
    // and we don't expand the states which doesn't lead to a valid grasp
    // If state is not valid and doesn't lead to optimal grasp, we ignore it


    if (currentState.second->id_ != 0 && !PredictFunctions::isValidState(currentState.second, _end_eff_type))
    {
      DBGAS("\nThis is an invalid search state <" << currentState.second->id_ << std::endl);

      DBGAS(*currentState.second.get());

      continue;
    }

    if (isLeaf(*currentState.second.get())) // First leave in tree is AStar answer
    {
      DBGAS("\nThe Goal is Found :) \n");

      reconstructPath(*currentState.second.get());

      _res = currentState.second;

      visitGoal = true;

      break;
    }

    successorsGenerator(currentState.second, &successors);

    for (auto& curr_successor : successors)
    {
      curr_successor->g_ = currentState.second->g_ + curr_successor->g_; // Updating g for successors

      // Updating h value for successors
      // h has dependency to successors, so for leaves is zero, since we don't have any sucessor
      if (isLeaf(*curr_successor.get()))
      {
        curr_successor->f_ = curr_successor->g_ + ( (1 + heuristic_weight_) * curr_successor->h_);

        DBGAS("\nAfter updating f value - Leaf\n_");
        DBGAS(*curr_successor.get() << std::endl);
      }
      else
      {
        successorsGenerator(curr_successor); // Look ahead for calculating h value
        curr_successor->f_ = curr_successor->g_ + ( (1 + heuristic_weight_) * curr_successor->h_);

        DBGAS("\nAfter updating f value - Look Ahead\n");
        DBGAS(*curr_successor.get() << std::endl);
      }

      unsigned target_id = curr_successor->id_;
      std::set<IDSearchStateSPtrPair>::iterator open_it = std::find_if(openList.begin(), openList.end(),
                  [&target_id](const IDSearchStateSPtrPair& _in)
                  {
                    return target_id == _in.second->id_;
                  });
      if (open_it != openList.end() && open_it->second->f_ < curr_successor->f_)
      {
        continue;
      }

      std::unordered_map<unsigned, SearchStateSPtr>::iterator close_it = closedList.find(curr_successor->id_);
      if (close_it != closedList.end() && close_it->second->f_ < curr_successor->f_)
      {
        continue;
      }

      openList.insert(make_pair(curr_successor->id_, curr_successor));
    }

    closedList[currentState.second->id_] = currentState.second;
  }

  if (visitGoal)
  {
    DBGAS("\nAStar DONE!!!\n");
    DBGAS("Number of Generated Nodes: " << numNodesCreated_ << std::endl);
    DBGAS("Number of Visited Nodes:   " << numNodesVisited_ << std::endl);
  }
  else
  {
    DBGAS("No Answer for this tree" << std::endl);
    DBGAS("Number of Generated Nodes: " << numNodesCreated_ << std::endl);
    DBGAS("Number of Visited Nodes:   " << numNodesVisited_ << std::endl);
  }
}


bool AStar::successorsGenerator(const SearchStateSPtr& _curr_state_ptr,
                                SearchStateSPtrSet* _successors)
{
  if (_successors == nullptr)
  {
    _successors = new SearchStateSPtrSet;
  }
  else
  {
    _successors->clear();
  }

  if (idSuccessorsMap_.count(_curr_state_ptr->id_))
  {
    *_successors = idSuccessorsMap_[_curr_state_ptr->id_];

    return true;
  }

  bool succ_res = false;

  switch(successor_type_)
  {
    case 0: // Random successor generator
      succ_res = SearchStateSuccessorsGenerators::randomSuccessorGenerator(_curr_state_ptr, branch_factor_,
                                                                           id_counter_, _successors);
      break;

    case 1: // Kmean successor generator
      succ_res = SearchStateSuccessorsGenerators::kmeanSuccessorGenerator(_curr_state_ptr, branch_factor_,
                                                                          id_counter_, _successors);
      break;
  }

  if (succ_res)
  {
    idSuccessorsMap_[_curr_state_ptr->id_] = *_successors;
    numNodesCreated_ += _successors->size();

    return true;
  }

  return false;
}


void AStar::reconstructPath(const SearchState& _goal)
{
  std::vector<unsigned> path_ids;
  std::vector<float> f_vals;

  path_ids.push_back(_goal.id_);
  f_vals.push_back(_goal.f_);

  SearchStateCSPtr curr = _goal.parent_;

  while (curr != nullptr)
  {
    path_ids.push_back(curr->id_);
    f_vals.push_back(curr->f_);
    curr = curr->parent_;
  }

  std::reverse(path_ids.begin(), path_ids.end());

  DBGAS("Final Path IDs:\n");
  for(const auto& id: path_ids)
  {
    DBGAS(id << "\t");
  }

  DBGAS(std::endl);

  DBGAS("Final Path Costs:\n");
  for(const auto& fval: f_vals)
  {
    DBGAS(fval << "\t");
  }
}
