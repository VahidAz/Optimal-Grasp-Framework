#ifndef __A_STAR_FUNCTORS_H_
#define __A_STAR_FUNCTORS_H_


#include "search_state.hpp"


typedef std::pair<unsigned, SearchStateSPtr> IDSearchStateSPtrPair;


// Functor for OpenList for Comparison by f
struct CompareIDSearchStateSPtrPairByF
{
  bool operator() (const IDSearchStateSPtrPair& _lhs, const IDSearchStateSPtrPair& _rhs)
  {
    return _lhs.second->f_ < _rhs.second->f_;
  }
};


#endif // __A_STAR_FUNCTORS_H_
