#ifndef __rrt_star_planner_h__
#define __rrt_star_planner_h__

#include <random>

#include <Eigen/Dense>
#include <cmath>
#include "planner_api.h"
#include "tree.h"
#include "types.h"
#include "utilities.h"

// Implement robotic planner with RRT* algorithm.
//  - Generate an initial joint position from the given start pose (see notes in plan
//  regarding this).
//  - Build up RRT tree with a slight variant of the classic RRT* algorithm:
//    - Since the goal space and joint space are different, we define two types of
//      metrics: state-state metric and state-pose metric:
//      - We use two types of inner-state metric, DistanceMetric and EdgeCostMetric,
//        for searching over close nodes and defining costs between nodes, respectively
//        (this is often the same metric in classic RRT*).
//      - When (occasionally) greedily searching towards the goal, branch off the node
//        that's "closest". This distance is defined in state-pose space
//        (DistanceToGoalMetric).
//  - When RRT* has finished, retrieve the best (according to EdgeCostMetric, of course)
//  solution from the tree and process it into a path of joint angles s.t.
//  (X_{i+1} - X_i).infinity_norm() <= resolution (no joint ever moves more than
//  resolution radians at any path step).
//
// Author notes:
// - There are some possible improvements to the code, I didn't make all of them because I
//   wanted to time-box the amount of time spent on this project and because this is, in
//   the end, a a toy problem / coding challenge.  Here are the improvements I would make
//   if deploying this code in a production environment:
//  -- All the TODOs scattered throughout the code specific to individual functions.
//  -- Better testing: I currently just randomly generate a bunch of poses (I've tested
//     with up to 100 randomly generated start/finish pose pairs), but it would be better
//     to have unit tests with with simple or pathological start/finish poses (e.g.
//     already at goal, start/finish are the same, both are invalid poses, etc.). Also,
//     unit testing for all of the metrics defined by the algorithm.
//  -- I've tried to define most algorithm parameters centrally, but a few more could
//     still be centralized.
//  -- Better documentation:
//     I like to document all my functions doxygen-style, e.g.:
//
//       /// \brief Generate initial joint position from initial pose.
//       /// \param start The initial pose.
//       /// \param success Written true if an initial joint position was found to match
//       ///        \p pose.
//       /// return The initial joint position. If !success, is unpopulated.
//       static Joint InitialX(const Pose& start, bool& success);

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
  void BuildTree(Joint X0, const Pose& goal, const double resolution, Tree& tree);

  // Rewire the RRT tree, examining nodes \p neighbor_idxs to see if node \p n_new_idx is
  // now a better parent than their existing parent.
  static void RewireTree(const std::vector<NodeID>& neighbor_idxs, const NodeID n_new_idx,
                         const NodeID best_parent_idx, const double resolution,
                         Tree& tree);

  // Find the best parent among \p neighbor_idxs for a node at \p X_new in \p tree.
  static std::pair<NodeID, double> BestParent(const NodeID nearest_node_idx,
                                              const Joint& X_new,
                                              const std::vector<NodeID>& neighbor_idxs,
                                              const double resolution, const Tree& tree);

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
