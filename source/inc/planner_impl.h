#ifndef __planner_impl_h__
#define __planner_impl_h__

#include <random>

#include <Eigen/Dense>
#include <cmath>
#include "planner_api.h"
#include "tree.h"
#include "types.h"
#include "utilities.h"

class RRTStarPlanner : public Planner {
 public:
  RRTStarPlanner(const double resolution)
      // Note: Looks like limits for all joints are [-175, +175] degrees
      // (aubo_i3_kinematics.cpp:632).
      : kSymmetricMaxJointAngle(Utilities::DegreesToRadians(175.0)),
        kMaxJointDisplacementBetweenNodes(Utilities::DegreesToRadians(5.0)),
        // Note: This number was just chosen by experimentation.
        // TODO Consider defining the radius algorithmically instead. Not sure how l-inf
        // norm as the distance metric affects how this is defined in the original RRT*
        // paper.
        kNearbyNodeRadius(Utilities::DegreesToRadians(15.0)) {
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

  /// Generate initial joint position from initial pose.
  static Joint InitialX(const Pose& start, bool& success);

  /// Given a planned tree, convert its best solution into a Path.
  static void ToPath(const Tree& tree, const double resolution, Path& path);

  /// Check for collisions along X0 -> Xf at resolution.
  static bool HasCollision(const Joint& X0, const Joint& Xf, const double resolution);

  // Create a high-resolution path along X0 -> X1 at most \p resolution
  // apart (each joint individually). Note: Adds X0, and Xi..., but not Xf, so guaranteed
  // to return a path length of at least 1.
  static Eigen::Matrix<double, Eigen::Dynamic, kDims> HighResolutionPath(
      const Joint& X0, const Joint& Xf, const double resolution);

  // Symmetric metric to measure search distance between joint positions. May differ from
  // EdgeCostMetric.
  static double DistanceMetric(const Joint& X0, const Joint& X1);

  // Symmetric metric for the cost to move the robot between joint positions. May differ
  // from DistanceMetric.
  static double EdgeCostMetric(const Joint& X0, const Joint& X1);

  // Symmetric metric to give a measure of how far a given joint position is from goal
  // pose. Since this function compares joint positions and poses, DistanceMetric is not
  // applicable.
  static double DistanceToGoalMetric(const Joint& X, const Pose& goal);

  // Returns true if \p position is within joint \p resolution of \p pose.
  static bool AtPose(const Joint& position, const Pose& pose, const double resolution);

  // Generate a (random?) target joint position to move towards within the joint state
  // space, given a greediness probability (to move directly towards the goal).
  Joint TargetX(const double greediness, const Pose& goal, const Joint& greedy_initial_X,
                bool& success);

  // Given a starting position and a target position, generate a new joint position at
  // most kMaxJointDisplacementBetweenNodes away (distance measured per-joint) in the
  // direction of target.
  Joint Steer(const Joint& X_start, const Joint& X_target) const;

  // Run the RRT star algorithm, starting from \p X0 and trying to get to \p goal,
  // guaranteeding path satisfiability with (joint space) \p resolution, and storing the
  // results in a tree.
  void RRT_star(Joint X0, const Pose& goal, const double resolution, Tree& tree);

 private:
  // The maximum displacement for each joint angle.
  const double kSymmetricMaxJointAngle;

  // The maximum joint displacement (per joint) allowed between RRT* search nodes.
  const double kMaxJointDisplacementBetweenNodes;

  // The radius to consider a node to be "nearby" while performing the search.
  const double kNearbyNodeRadius;

  // For generating random numbers.
  std::default_random_engine engine_;
  std::uniform_real_distribution<double> uniform_distribution_;
};

#endif
