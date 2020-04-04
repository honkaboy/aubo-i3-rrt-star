#ifndef __types_h__
#define __types_h__

#include <Eigen/Dense>

static constexpr size_t kDims = 6;
typedef uint32_t NodeID;
typedef Eigen::Matrix<double, 1, kDims> Joint;

#endif
