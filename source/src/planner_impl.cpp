#include "planner_impl.h"

#include <Eigen/LU>
#include <cmath>
#include <iostream>
#include "planner_api.h"
#include "robot_api.h"
#include "tree.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// TODO cleanup:
// - go though and make things references, const refs where possible.
// - make functions static or const where possible
// - Dox for class
// - Replace asserts.
// - Loops: iterators
// - Make consts constexpr where possible.
// - Integration test with a bunch of different poses.

double PlannerImpl::DistanceMetric(const VectorXd& X0, const VectorXd& X1) {
  // Use the inf-norm for search distance to reflect that each joint is independent.
  return (X0 - X1).lpNorm<Eigen::Infinity>();
}

double PlannerImpl::EdgeCostMetric(const VectorXd& X0, const VectorXd& X1) {
  // Use the 2-norm for edge costs between nodes to reflect that we want to minimize total
  // joint movement.
  return (X0 - X1).lpNorm<2>();
}

double PlannerImpl::DistanceToGoalMetric(const VectorXd& X, const Pose& goal) {
  // Got this metric from https://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf eqn. 4,
  // which in turn got it from https://doi.org/10.1115/1.2826116
  // metric = sqrt(a * norm(log(R2 / R1))**2 + b * norm(t2 - t1)**2)
  const Pose::AffineTransform_t T_X = RobotAPI::forward_kinematics(X);
  const Pose::AffineTransform_t T_goal = goal.AffineTransform();
  const double a = 1.0;
  const double b = 1.0;

  // Calculate log(R2/R1). From
  // https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
  const Eigen::AngleAxisd R(T_X.rotation().inverse() * T_goal.rotation());
  const double angle = R.angle();
  Matrix3d log_R = Matrix3d::Zero();
  // calculate with deg2rad
  if (angle > 0.001 * M_PI / 180.0) {
    log_R = angle / (2 * std::sin(angle)) *
            (R.toRotationMatrix() - R.toRotationMatrix().transpose());
  }

  // Calculate the metric.
  const double metric =
      std::sqrt(a * log_R.squaredNorm() +
                b * (T_X.translation() - T_goal.translation()).squaredNorm());
  return metric;
}

VectorXd PlannerImpl::InitialX(const Pose& start, bool& success) {
  /// Use the inverse kinematics starting from the joint origin to obtain the initial
  /// joint position.
  VectorXd origin(kDims);
  origin << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  VectorXd initial_joints =
      RobotAPI::inverse_kinematics(start.AffineTransform(), origin, success);
  return initial_joints;
}

Path PlannerImpl::ToPath(const Tree& tree, const double resolution) const {
  const std::vector<NodeID> solution = tree.Solution();
  Eigen::Matrix<double, Eigen::Dynamic, kDims> joint_positions;

  Path path;

  std::cout << "HERE" << __LINE__ << "\n" << std::endl;
  for (auto it = solution.begin(); it != solution.end() - 1; ++it) {
    // Only include the last joint position on the last iteration through. Otherwise we'd
    // double-count intermediate joint positions.
    const bool include_last = it == (solution.end() - 1);
    const VectorXd& X0 = tree.GetNode(*it).position;
    const VectorXd& X1 = tree.GetNode(*(it + 1)).position;
    std::cout << "HERE" << __LINE__ << "\n" << std::endl;
    path.joint_positions << HighResolutionPath(X0, X1, resolution);
  }
  std::cout << "HERE" << __LINE__ << "\n" << std::endl;

  // Convert to path object and return;
  return path;
}

// Default definition of a virtual planner
Path PlannerImpl::plan(const Pose& start, const Pose& end, double resolution,
                       bool& plan_ok) {
  Path path;

  /// NOTE: This function's API is a little strange, because we should expect to already
  /// know the robot's initial configuration (initial joint displacements) if we're
  /// planning from a starting pose. Since we're missing this information, I've defined
  /// this function here that generates the initial joint angles which are otherwise
  /// likely underconstrained (assuming inverse kinematics pose -> joints is a many-to-one
  /// function.
  VectorXd initial_joints = InitialX(start, plan_ok);
  if (!plan_ok) {
    std::cout << "Error: Failed to obtain initial joint angles." << std::endl;
  } else {
    // NOTE: We don't just calculate a target joint position because there is likely a set
    // of joint positions that reach end, and we don't want to overconstrain the planner.
    // TODO Pass tree by reference instead
    // TODO return !plan_ok if we failed to find a path.
    std::cout << "HERE" << __LINE__ << "\n" << std::endl;
    Tree tree = RRT_star(initial_joints, end, resolution);

    // Print search report. TODO
    // tree.Report();

    std::cout << "HERE" << __LINE__ << "\n" << std::endl;
    // Process tree into path.
    path = ToPath(tree, resolution);
  }
  return path;
}

// TODO dox Creates a high-resolution path along X0 -> X1 at most \p resolution
// apart (each joint individually). Note: Adds X0, and Xi..., but not Xf, so guaranteed to
// return a path length of at least 1.c
Eigen::Matrix<double, Eigen::Dynamic, kDims> PlannerImpl::HighResolutionPath(
    const VectorXd& X0, const VectorXd& Xf, const double resolution) const {
  Eigen::Matrix<double, Eigen::Dynamic, kDims> points;
  std::cout << "HERE" << __LINE__ << "\n" << std::endl;

  const double linf_dist = (X0 - Xf).lpNorm<Eigen::Infinity>();
  // TODO remove sanity check? Not necessary to double-check the norm.
  assert(linf_dist >= 0.0);
  const int path_count = std::max(static_cast<int>(std::ceil(linf_dist / resolution)), 1);
  std::cout << "HERE" << __LINE__ << "\n" << std::endl;
  const VectorXd dX = (Xf - X0) / path_count;
  for (int i = 0; i < path_count; ++i) {
    const VectorXd Xi = X0 + i * dX;
    std::cout << "HERE" << __LINE__ << "\n" << std::endl;
    points << Xi;
  }
}

bool PlannerImpl::HasCollision(const VectorXd& X0, const VectorXd& Xf,
                               const double resolution) const {
  Eigen::Matrix<double, Eigen::Dynamic, kDims> points =
      HighResolutionPath(X0, Xf, resolution);

  // Check for collisions <= resolution for every joint individually so we
  // can guarantee the path between nodes is collision-free up to resolution.
  // TODO this is pretty inefficient because Eigen matrices are stored column-wise. There
  // a better way to iterate over rows?
  for (size_t i = 0; i < points.rows(); ++i) {
    if (RobotAPI::in_collision(points.row(i))) {
      return true;
    }
  }

  // Note: points does not contain Xf.
  return RobotAPI::in_collision(Xf);
}

bool PlannerImpl::AtPose(const VectorXd& position, const Pose& pose,
                         const double resolution) {
  bool success = false;
  const VectorXd goal_X =
      RobotAPI::inverse_kinematics(pose.AffineTransform(), position, success);
  if (success) {
    // DistanceMetric < resolution ensures that all joints can move to goal within
    // resolution independently.
    success = DistanceMetric(position, goal_X) < resolution;
  }
  return success;
}

VectorXd PlannerImpl::TargetX(const double greediness, const Pose& goal,
                              const VectorXd& greedy_initial_X) {
  VectorXd target(kDims);
  if (uniform_distribution_(engine_) < greediness) {
    // In the greedy case, go directly to goal. Inverse kinematics specifies the goal
    // state from a given greedy_initial_X.
    bool success = false;
    target =
        RobotAPI::inverse_kinematics(goal.AffineTransform(), greedy_initial_X, success);
    // TODO something smarter than assert here.
    assert(success);
  } else {
    // Otherwise, randomly select a target in the state space.
    // Note: This works because the min/max joint angles are symmetric [-max, max] and
    // Random() generates x \in [-1, 1].
    target = VectorXd::Random(kDims) * kSymmetricMaxJointAngle;
  }
  return target;
}

VectorXd PlannerImpl::Steer(const VectorXd& X_root, const VectorXd& X_goal) {
  // TODO update comment since linf is no longer explicit
  // Steer in the direction of X_goal without any single joint exceeding dx =
  // kMaxJointDisplacementBetweenNodes and without overshooting the goal.
  VectorXd steered(kDims);
  const double distance = DistanceMetric(X_root, X_goal);
  if (distance < kMaxJointDisplacementBetweenNodes) {
    steered = X_goal;
  } else {
    // Since DistanceMetric should be a norm, and kMaxJointDisplacementBetweenNodes must
    // logically be positive, distance should always > 0 at this point.
    assert(distance > 0.0);
    const double du = kMaxJointDisplacementBetweenNodes / distance;
    steered = X_root + (X_goal - X_root) * du;
  }
  return steered;
}

// TODO could be defined at class construction instead of as a function?
double PlannerImpl::CalculateNearRadius() {
  // Since we're using l-inf norm for the distance metric, just search within... oh, I
  // dunno, 3x max distance between nodes?
  // TODO Define this parameter in a central location.
  return 3.0 * kMaxJointDisplacementBetweenNodes;
}

Tree PlannerImpl::RRT_star(VectorXd X0, const Pose& goal, const double resolution) {
  // TODO Define this parameter in a central location.
  const size_t kMaxIterations = 1000;

  const auto cost_to_go = [&goal](const VectorXd& X) {
    return DistanceToGoalMetric(X, goal);
  };

  const double root_node_cost = 0.0;
  Node root(X0, Tree::kNone, root_node_cost, cost_to_go(X0));
  Tree tree(root, DistanceMetric, kMaxIterations);

  std::cout << "HERE" << __LINE__ << "\n" << std::endl;
  // TODO Handle case where root is already at goal.
  // TODO Handle case where root is in collision.
  // TODO Handle case where goal is in collision.

  // Iterate kMaxIterations -1 times since we already have 1 node in the tree.
  // TODO Maybe fill up the whole tree? This wouldn't do it because of collisions.
  std::cout << "HERE" << __LINE__ << "\n" << std::endl;
  for (size_t i = 0; i < kMaxIterations - 1; ++i) {
    // Occasional greedy choice directly towards goal.
    const double greediness = 0.1;
    VectorXd X_target = TargetX(greediness, goal, tree.GetBestNodePosition());

    // From the "randomly" generated target state, generate a new candidate state.
    // TODO don't build off of goal nodes?
    const NodeID nearest_node_idx = tree.nearest(X_target);
    const VectorXd X_nearest = tree.GetNode(nearest_node_idx).position;
    const VectorXd X_new = Steer(X_nearest, X_target);

    if (!HasCollision(X_nearest, X_new, resolution)) {
      const double radius = CalculateNearRadius();
      // Note that tree does not yet contain X_new.
      const std::vector<NodeID> neighbor_idxs = tree.near_idxs(X_new, radius);
      // Connect X_new to best "near" node. Cost to traverse is defined by nearest()'s
      // distance metric.
      NodeID best_parent_idx = nearest_node_idx;
      const Node n_nearest = tree.GetNode(nearest_node_idx);
      // Minimum cost to get to X_new through neighbors.
      double cost_through_best_parent =
          n_nearest.cost + EdgeCostMetric(n_nearest.position, X_new);
      for (const NodeID neighbor_idx : neighbor_idxs) {
        const Node n_neighbor = tree.GetNode(neighbor_idx);
        const double new_cost_through_neighbor =
            n_neighbor.cost + EdgeCostMetric(n_neighbor.position, X_new);
        if (new_cost_through_neighbor < cost_through_best_parent &&
            !HasCollision(n_neighbor.position, X_new, resolution)) {
          best_parent_idx = neighbor_idx;
          cost_through_best_parent = new_cost_through_neighbor;
        }
      }

      // Add X_new to tree through best "near" node.
      const Node n_new =
          Node(X_new, best_parent_idx, cost_through_best_parent, cost_to_go(X_new));
      NodeID n_new_idx = tree.Add(n_new, AtPose(n_new.position, goal, resolution));
      assert(n_new_idx != Tree::kNone);

      // Connect all neighbors of X_new to X_new if that path cost is less.
      for (const NodeID neighbor_idx : neighbor_idxs) {
        // TODO don't search over best_cost_idx (the best parent for n_new)
        Node n_neighbor = tree.GetNode(neighbor_idx);
        const double neighbor_cost_through_new =
            n_new.cost + EdgeCostMetric(n_neighbor.position, n_new.position);
        if (neighbor_cost_through_new < n_neighbor.cost and
            !HasCollision(n_new.position, n_neighbor.position, resolution)) {
          // Best path for neighbor is now through X_new
          n_neighbor.parent = n_new_idx;
          n_neighbor.cost = neighbor_cost_through_new;
          // Update the neighbor node in the tree.
          tree.SetNode(neighbor_idx, n_neighbor);
        }
      }
    }
  }
  std::cout << "HERE" << __LINE__ << "\n" << std::endl;

  return tree;
}

