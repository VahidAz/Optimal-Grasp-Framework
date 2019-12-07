#ifndef __COMMON_DEFS_HPP_
#define __COMMON_DEFS_HPP_


#include <limits>


#define MIN_VALID_FINGERS_NUM 2
#define MIN_VALID_BRANCH_FACTOR 2

#define NEG_INFINITY std::numeric_limits<float>::min()

#define ETA_NORMAL_WEIGHT 2

#define THRESHOLD_IGNORE_DIFF_QUALITY 0.001


typedef enum
{
  parallel_jaw = 0

} END_EFFECTOR_T;


#endif // __COMMON_DEFS_HPP_
