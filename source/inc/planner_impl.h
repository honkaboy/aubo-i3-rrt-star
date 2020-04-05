#ifndef __planner_impl_h__
#define __planner_impl_h__

#include <random>

#include <Eigen/Dense>
#include <cmath>
#include "planner_api.h"
#include "tree.h"
#include "types.h"
#include "utilities.h"

class PlannerImpl : public Planner {
 public:
  PlannerImpl(const double resolution)
      // Looks like limits for all joints are [-175, +175] degrees
      // (aubo_i3_kinematics.cpp:632).
      : kSymmetricMaxJointAngle(Utilities::DegreesToRadians(175.0)),
        kMaxJointDisplacementBetweenNodes(Utilities::DegreesToRadians(5.0)) {
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

  static Joint InitialX(const Pose& start, bool& success);

  static void ToPath(const Tree& tree, const double resolution, Path& path);

  static bool HasCollision(const Joint& X0, const Joint& Xf, const double resolution);

  static Eigen::Matrix<double, Eigen::Dynamic, kDims> HighResolutionPath(
      const Joint& X0, const Joint& Xf, const double resolution);

  static double DistanceMetric(const Joint& X0, const Joint& X1);

  static double EdgeCostMetric(const Joint& X0, const Joint& X1);

  static double DistanceToGoalMetric(const Joint& X, const Pose& goal);

  static bool AtPose(const Joint& position, const Pose& pose, const double resolution);

  Joint TargetX(const double greediness, const Pose& goal, const Joint& greedy_initial_X);

  Joint Steer(const Joint& X_root, const Joint& X_goal) const;

  double CalculateNearRadius() const;

  void RRT_star(Joint X0, const Pose& goal, const double resolution, Tree& tree);

 private:
  const double kSymmetricMaxJointAngle;
  const double kMaxJointDisplacementBetweenNodes;
  std::default_random_engine engine_;
  std::uniform_real_distribution<double> uniform_distribution_;
};

#endif
