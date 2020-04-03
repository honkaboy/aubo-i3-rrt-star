#ifndef __planner_impl_h__
#define __planner_impl_h__

#include <random>

#include <Eigen/Dense>
#include <cmath>
#include "planner_api.h"
#include "types.h"

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// double DegreesToRadians(const double degrees) { return degrees / 180.0 * M_PI; }
// double RadiansToDegrees(const double radians) { return radians / M_PI * 180.0; }

class PlannerImpl : public Planner {
 public:
  PlannerImpl(const double resolution)
      : kResolution(resolution),
        // Looks like limits for all joints are [-175, +175] degrees
        // (aubo_i3_kinematics.cpp:632).
        // TODO maybe define these in a prettier way.
        kSymmetricMaxJointAngle(175.0 / 180.0 * M_PI),
        kMaxJointDisplacementBetweenNodes(5.0 / 180.0 * M_PI) {
    // Initialize RNG.
    uniform_distribution_ = std::uniform_real_distribution<double>(0, 1);
  }

  /// \brief Function all planners will override.
  /// \param[in] start - the starting position of the path
  /// \param[in] end - the ending position of the path
  /// \param[out] plan_ok - by reference flag where true means the plan
  /// was successfully planned, false means a plan could not be found
  /// \return Path object representing the planned path
  Path plan(const Pose& start, const Pose& end, double resolution, bool& plan_ok) final;

  bool HasCollision(const VectorXd& X0, const VectorXd& X1);

  static double DistanceMetric(const VectorXd& X0, const VectorXd& X1);

  static double CostMetric(const VectorXd& X0, const VectorXd& X1);

  bool AtGoal(const VectorXd& position);

  VectorXd TargetX(const double greediness, const VectorXd goal);

  VectorXd Steer(const VectorXd& X_root, const VectorXd& X_goal);

  double CalculateNearRadius();

  void RRT_star(VectorXd X0);

 private:
  // Looks like limits for all joints are [-175, +175] degrees
  // (aubo_i3_kinematics.cpp:632).
  const size_t kDims = 6;  // TODO This could be a static constexpr
  const double kResolution;
  const double kSymmetricMaxJointAngle;
  const double kMaxJointDisplacementBetweenNodes;
  std::default_random_engine engine_;
  std::uniform_real_distribution<double> uniform_distribution_;
};

#endif
