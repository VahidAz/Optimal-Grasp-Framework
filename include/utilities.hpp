#ifndef __UTILITIES_H_
#define __UTILITIES_H_


#include <math.h>

#include <pcl/common/eigen.h>

#include "search_state.hpp"
#include "pcl_defs.hpp"


#define TORADIAN(x) x*M_PI/180.0


class Utilities
{
  public:
    Utilities() = delete;
    ~Utilities() = delete;

    Utilities(const Utilities&) = delete;
    Utilities& operator =(const Utilities&) = delete;

    static inline float euclDist3D(const float& _x1, const float& _y1, const float& _z1,
                                   const float& _x2, const float& _y2, const float& _z2)
    {
      return sqrt(squaredDist3D(_x1, _y1, _z1, _x2, _y2, _z2));
    }

    static inline float squaredDist3D(const float& _x1, const float& _y1, const float& _z1,
                                      const float& _x2, const float& _y2, const float& _z2)
    {
      return (pow(_x1 - _x2, 2.0) + pow(_y1 - _y2, 2.0) + pow(_z1 - _z2, 2.0));
    }

    static inline float vecMagnitude3D(const float& _x, const float& _y, const float& _z)
    {
      return squaredDist3D(_x, _y, _z, 0, 0, 0);
    }

    static inline float dotProduct(const float& _x1, const float& _y1, const float& _z1,
                                   const float& _x2, const float& _y2, const float& _z2)
    {
      Eigen::Vector3f firstVec(_x1, _y1, _z1);
      Eigen::Vector3f secVec(_x2, _y2, _z2);

      return firstVec.dot(secVec);
    }

    static inline void cart_product(const SuperContactSet_Set& _in,
                                    SuperContactSet_Set& _res)
    {
      _res = {{}};

      for (const auto& u : _in)
      {
        SuperContactSet_Set r;

        for (const auto& x : _res)
        {
          for (const auto& y : u)
          {
            r.push_back(x);
            r.back().push_back(y);
          }
        }
        _res = r;
      }
    }

    static inline void enumVecs(const ContactPointSet_Set& _inVecs,
                                size_t _vecIndex, std::vector<unsigned> _enumSoFar,
                                ContactPointSet_Set& _result)
    {
      if (_vecIndex >= _inVecs.size())
      {
        _result.push_back(_enumSoFar);

        return;
      }

      for (size_t i = 0; i < _inVecs[_vecIndex].size(); ++i)
      {
        _enumSoFar.push_back(_inVecs[_vecIndex][i]);

        enumVecs(_inVecs, _vecIndex + 1, _enumSoFar, _result);

        _enumSoFar.pop_back();
      }
    }
};


#endif // __UTILITIES_H_
