#ifndef __planner_impl_h__
#define __planner_impl_h__

#include "planner_api.h"
#include "types.h"

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

class PlannerImpl : public Planner {
 public:
  PlannerImpl() {}

  /// \brief Function all planners will override.
  /// \param[in] start - the starting position of the path
  /// \param[in] end - the ending position of the path
  /// \param[in] resolution - the minimum spacing between incremental path positions (how
  /// fine the path is)
  /// \param[out] plan_ok - by reference flag where true means the plan
  /// was successfully planned, false means a plan could not be found
  /// \return Path object representing the planned path
  virtual Path plan(const Pose& start, const Pose& end, double resolution,
            bool& plan_ok) override;

  static double DistanceMetric(const VectorXd&, const VectorXd&);
};

#endif
