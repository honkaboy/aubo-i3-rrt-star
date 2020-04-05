#include "planner_impl.h"

#include <Eigen/LU>
#include <cmath>
#include <iostream>
#include "planner_api.h"
#include "robot_api.h"
#include "tree.h"

// TODO cleanup:
// - go though and make things references, const refs where possible.
// - Dox for class
// - Loops: iterators
// - Make consts constexpr where possible.
// - Integration test with a bunch of different poses.

Joint PlannerImpl::InitialX(const Pose& start, bool& success) {
  /// Use the inverse kinematics starting from the joint origin to obtain the initial
  /// joint position.
  Joint origin(kDims);
  origin << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Joint initial_joints =
      RobotAPI::inverse_kinematics(start.AffineTransform(), origin, success);
  return initial_joints;
}

double PlannerImpl::DistanceMetric(const Joint& X0, const Joint& X1) {
  // Use the inf-norm for search distance to reflect that each joint is independent.
  return (X0 - X1).lpNorm<Eigen::Infinity>();
}

double PlannerImpl::EdgeCostMetric(const Joint& X0, const Joint& X1) {
  // Use the 2-norm for edge costs between nodes to reflect that we want to minimize total
  // joint movement.
  return (X0 - X1).lpNorm<2>();
}

double PlannerImpl::DistanceToGoalMetric(const Joint& X, const Pose& goal) {
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
  Eigen::Matrix3d log_R = Eigen::Matrix3d::Zero();
  if (angle > Utilities::DegreesToRadians(0.001)) {
    log_R = angle / (2 * std::sin(angle)) *
            (R.toRotationMatrix() - R.toRotationMatrix().transpose());
  }

  // Calculate the metric.
  const double metric =
      std::sqrt(a * log_R.squaredNorm() +
                b * (T_X.translation() - T_goal.translation()).squaredNorm());
  return metric;
}

void PlannerImpl::ToPath(const Tree& tree, const double resolution, Path& path) {
  const std::vector<NodeID> solution = tree.Solution();

  if (solution.size() >= 1) {
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, kDims>> edge_paths;

    size_t num_joint_positions = 0;
    for (auto it = solution.begin(); it != solution.end() - 1; ++it) {
      // Only include the last joint position on the last iteration through. Otherwise
      // we'd double-count intermediate joint positions.
      const Joint& X0 = tree.GetNode(*it).position;
      const Joint& X1 = tree.GetNode(*(it + 1)).position;
      edge_paths.push_back(HighResolutionPath(X0, X1, resolution));

      // Compute the number of rows total in our joint path matrix.
      num_joint_positions += edge_paths.back().rows();
    }
    // Make space for the final row.
    num_joint_positions += 1;

    // Store all calculated joint positions in the path object.
    path.joint_positions.resize(num_joint_positions, kDims);
    size_t current_row = 0;
    const size_t column = 0;
    for (size_t i = 0; i < edge_paths.size(); ++i) {
      const Eigen::Matrix<double, Eigen::Dynamic, kDims>& edge_path = edge_paths[i];
      const size_t num_rows = edge_path.rows();
      path.joint_positions.block(current_row, column, num_rows, kDims) = edge_path;
      current_row += num_rows;
    }
    const Joint final_joint_position = tree.GetNode(solution.back()).position;
    path.joint_positions.row(current_row++) = final_joint_position;

    // All rows should have been filled, were they?
    if (current_row != path.joint_positions.rows()) {
      throw std::logic_error("Failed to populate joint_positions.");
    }
  }

  return;
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
  Joint initial_joints = InitialX(start, plan_ok);
  if (!plan_ok) {
    std::cout << "Error: Failed to obtain initial joint angles." << std::endl;
  } else {
    // NOTE: We don't just calculate a target joint position because there is likely a set
    // of joint positions that reach end, and we don't want to overconstrain the planner.
    // TODO return !plan_ok if we failed to find a path.
    // TODO Define this parameter in a central location.
    const size_t kMaxNodes = 1000;

    Tree tree(DistanceMetric, kMaxNodes);
    RRT_star(initial_joints, end, resolution, tree);

    // Print search report. TODO
    // tree.Report();

    // Process tree into path.
    ToPath(tree, resolution, path);
  }
  return path;
}

Eigen::Matrix<double, Eigen::Dynamic, kDims> PlannerImpl::HighResolutionPath(
    const Joint& X0, const Joint& Xf, const double resolution) {
  Eigen::Matrix<double, Eigen::Dynamic, kDims> points;

  // Use L-inf norm so no joint moves more than resolution per path step.
  const double linf_dist = (X0 - Xf).lpNorm<Eigen::Infinity>();

  // Number of path steps that must be used to get between the points with \p resolution.
  const int path_steps = std::max(static_cast<int>(std::ceil(linf_dist / resolution)), 1);
  const Joint dX = (Xf - X0) / path_steps;
  points.resize(path_steps, kDims);
  for (int i = 0; i < path_steps; ++i) {
    const Joint Xi = X0 + i * dX;
    points.row(i) = Xi;
  }
  return points;
}

bool PlannerImpl::HasCollision(const Joint& X0, const Joint& Xf,
                               const double resolution) {
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

bool PlannerImpl::AtPose(const Joint& position, const Pose& pose,
                         const double resolution) {
  bool success = false;
  const Joint goal_X =
      RobotAPI::inverse_kinematics(pose.AffineTransform(), position, success);
  if (success) {
    // DistanceMetric < resolution ensures that all joints can move to goal within
    // resolution independently.
    success = DistanceMetric(position, goal_X) < resolution;
  }
  return success;
}

Joint PlannerImpl::TargetX(const double greediness, const Pose& goal,
                           const Joint& greedy_initial_X, bool& success) {
  Joint target(kDims);
  if (uniform_distribution_(engine_) < greediness) {
    // In the greedy case, go directly to goal. Inverse kinematics specifies the goal
    // state from a given greedy_initial_X.
    target =
        RobotAPI::inverse_kinematics(goal.AffineTransform(), greedy_initial_X, success);
  } else {
    // Otherwise, randomly select a target in the state space.
    // Note: This works because the min/max joint angles are symmetric [-max, max] and
    // Random() generates x \in [-1, 1].
    target = Joint::Random(kDims) * kSymmetricMaxJointAngle;
    success = true;
  }
  return target;
}

Joint PlannerImpl::Steer(const Joint& X_start, const Joint& X_target) const {
  // TODO update comment since linf is no longer explicit
  // Steer in the direction of X_target without any single joint exceeding dx =
  // kMaxJointDisplacementBetweenNodes and without overshooting the goal.
  Joint steered(kDims);
  const double distance = DistanceMetric(X_start, X_target);
  if (distance < kMaxJointDisplacementBetweenNodes) {
    steered = X_target;
  } else {
    if (distance <= 0.0) {
      // Since DistanceMetric should be a norm, and kMaxJointDisplacementBetweenNodes must
      // logically be positive, distance should always > 0 at this point.
      throw std::logic_error(
          "Distance between start and target nodes was unexpectedly nonpositive.");
    }
    const double du = kMaxJointDisplacementBetweenNodes / distance;
    steered = X_start + (X_target - X_start) * du;
  }
  return steered;
}

void PlannerImpl::RRT_star(Joint X0, const Pose& goal, const double resolution,
                           Tree& tree) {
  const auto distance_to_goal = [&goal](const Joint& X) {
    return DistanceToGoalMetric(X, goal);
  };

  const double root_node_cost = 0.0;
  Node root(X0, Tree::kNone, root_node_cost, distance_to_goal(X0));
  tree.Add(root);

  // TODO Handle case where root is already at goal.
  // TODO Handle case where root is in collision.
  // TODO Handle case where goal is in collision.

  // TODO store in a central location.
  const size_t kMaxIterations = 10000;
  for (size_t i = 0; i < kMaxIterations && !tree.IsFull(); ++i) {
    // Occasional greedy choice directly towards goal.
    const double greediness = 0.1;
    bool success;
    Joint X_target = TargetX(greediness, goal, tree.GetBestNodePosition(), success);
    if (!success) {
      // This may occasionally fail since since TargetX relies on inverse kinematics to
      // get the goal joint positions. We just restart the loop and try again.
      continue;
    }

    // From the "randomly" generated target state, generate a new candidate state.
    // TODO Make more efficient: don't build off of goal nodes
    const NodeID nearest_node_idx = tree.nearest(X_target);
    const Joint X_nearest = tree.GetNode(nearest_node_idx).position;
    const Joint X_new = Steer(X_nearest, X_target);

    if (!HasCollision(X_nearest, X_new, resolution)) {
      // Note that tree does not yet contain X_new.
      const std::vector<NodeID> neighbor_idxs = tree.near_idxs(X_new, kNearbyNodeRadius);
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
          Node(X_new, best_parent_idx, cost_through_best_parent, distance_to_goal(X_new));
      NodeID n_new_idx = tree.Add(n_new, AtPose(n_new.position, goal, resolution));

      if (n_new_idx == Tree::kNone) {
        throw std::logic_error("Tree unexpectedly ran out of room to store nodes.");
      }

      // Connect all neighbors of n_new to n_new if that path cost is less.
      for (const NodeID neighbor_idx : neighbor_idxs) {
        // TODO Make more efficient: don't search over best_cost_idx (the best parent for
        // n_new)
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

  return;
}

