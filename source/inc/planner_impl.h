#ifndef __planner_impl_h__
#define __planner_impl_h__

#include <random>

#include <Eigen/Dense>
#include <cmath>
#include "planner_api.h"
#include "tree.h"
#include "types.h"

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// double DegreesToRadians(const double degrees) { return degrees / 180.0 * M_PI; }
// double RadiansToDegrees(const double radians) { return radians / M_PI * 180.0; }

static constexpr size_t kDims = 6;

class PlannerImpl : public Planner {
 public:
  PlannerImpl(const double resolution)
      // Looks like limits for all joints are [-175, +175] degrees
      // (aubo_i3_kinematics.cpp:632).
      // TODO maybe define these in a prettier way.
      : kSymmetricMaxJointAngle(175.0 / 180.0 * M_PI),
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

  VectorXd InitialX(const Pose& start, bool& success);

  Path ToPath(const Tree& tree, const double resolution) const;

  bool HasCollision(const VectorXd& X0, const VectorXd& Xf,
                    const double resolution) const;

  Eigen::Matrix<double, Eigen::Dynamic, kDims> HighResolutionPath(
      const VectorXd& X0, const VectorXd& Xf, const double resolution) const;

  static double DistanceMetric(const VectorXd& X0, const VectorXd& X1);

  static double EdgeCostMetric(const VectorXd& X0, const VectorXd& X1);

  static double DistanceToGoalMetric(const VectorXd& X, const Pose& goal);

  bool AtPose(const VectorXd& position, const Pose& pose, const double resolution);

  VectorXd TargetX(const double greediness, const Pose& goal,
                   const VectorXd& greedy_initial_X);

  VectorXd Steer(const VectorXd& X_root, const VectorXd& X_goal);

  double CalculateNearRadius();

  Tree RRT_star(VectorXd X0, const Pose& goal, const double resolution);

 private:
  const double kSymmetricMaxJointAngle;
  const double kMaxJointDisplacementBetweenNodes;
  std::default_random_engine engine_;
  std::uniform_real_distribution<double> uniform_distribution_;
};

#endif
