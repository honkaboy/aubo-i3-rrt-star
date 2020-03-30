#include "planner_impl.h"

#include "robot_api.h"
#include "tree.h"

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

double PlannerImpl::DistanceMetric(const VectorXd& X0, const VectorXd& X1) {
  // TODO 1-norm is probably more appropriate.
  // return (X0 - X1).lpNorm<1>();
  return (X0 - X1).norm();
}

VectorXd InitialPosition(const Pose& start) {
  VectorXd init(6);
  init << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  return init;
}

// Default definition of a virtual planner
Path PlannerImpl::plan(const Pose& start, const Pose& end, double resolution,
                       bool& plan_ok) {
  /// TODO figure out how to properly incorporate the resolution.
  /// TODO The robot's initial joint positions should be specified, as supplying simply
  /// the pose to begin likely underconstrains the problem (assuming inverse kinematics
  /// pose -> joints is a many-to-one function.
  VectorXd initial_joints = InitialPosition(start);
  // Bogus isn't actually used.
  RRT_star(initial_joints);

  plan_ok = false;
  return Path();
}

bool PlannerImpl::HasCollision(const VectorXd& X0, const VectorXd& X1) {
  if (DistanceMetric(X0, X1) < kPrecision) {
    return in_collision(X0);
  }

  const double path_count = std::ceil(DistanceMetric(X0, X1) / kPrecision);
  const VectorXd dX = (X1 - X0) / path_count;

  for (size_t i = 0; i < path_count; ++i) {
    const VectorXd Xi = X0 + i * dX;
    if (in_collision(Xi)) {
      return true;
    }
  }
  if (in_collision(X1)) {
    return true;
  }
}

bool PlannerImpl::AtGoal(const VectorXd& position) {
  VectorXd goal(6);
  goal << 0.15, 0.15, 0.15, 0.15, 0.15, 0.15;
  return DistanceMetric(position, goal) < kPrecision;

  // TODO
  // Either (1) near position + rotation or (2) near target joint angles. 1 seems better.
}

VectorXd PlannerImpl::RandomX() {
  // This works because the min/max joint angles are symmetric [-max, max] and Random()
  // generates [-1, 1].
  VectorXd random_point = VectorXd::Random(kDims) * kSymmetricMaxJointAngle;
  return random_point;
}

VectorXd PlannerImpl::Steer(const VectorXd& X_root, const VectorXd& X_goal) {
  const double distance = DistanceMetric(X_root, X_goal);
  // TODO calculate better
  const double du = new_node_distance_ / distance;
  const VectorXd steered = X_root + (X_goal - X_root) * du;
  return steered;
}

double CalculateNearRadius() {
  // TODO Actually calculate.
  return 0.1;
}

void PlannerImpl::RRT_star(VectorXd X0) {
  const size_t max_nodes = 1000;

  Node root(X0, Tree::kNone, 0.0);
  Tree tree(root, DistanceMetric, max_nodes);

  // TODO Handle case where root is already at goal.
  // TODO Handle case where root is in collision.
  // TODO Handle case where goal is in collision.
  // TODO go though and make things references where possible.

  for (size_t i; i < max_nodes; ++i) {
    // TODO add occasional greedy choice directly towards Xf
    VectorXd X_random = RandomX();
    const NodeID nearest_node_idx = tree.nearest(X_random);
    const VectorXd X_nearest = tree.GetNode(nearest_node_idx).position;
    const VectorXd X_new = Steer(X_nearest, X_random);

    if (!HasCollision(X_nearest, X_new)) {
      const double radius = CalculateNearRadius();
      // Note that tree does not yet contain X_new.
      const std::vector<NodeID> neighbor_idxs = tree.near_idxs(X_new, radius);
      // Connect X_new to best "near" node. Cost to traverse is defined by nearest()'s
      // distance metric.
      NodeID best_parent_idx = nearest_node_idx;
      const Node n_nearest = tree.GetNode(nearest_node_idx);
      // Minimum cost to get to X_new through neighbors.
      double cost_through_best_parent =
          n_nearest.cost + DistanceMetric(n_nearest.position, X_new);
      for (const NodeID neighbor_idx : neighbor_idxs) {
        const Node n_neighbor = tree.GetNode(neighbor_idx);
        const double new_cost_through_neighbor =
            n_neighbor.cost + DistanceMetric(n_neighbor.position, X_new);
        if (new_cost_through_neighbor < cost_through_best_parent &&
            !HasCollision(n_neighbor.position, X_new)) {
          best_parent_idx = neighbor_idx;
          cost_through_best_parent = new_cost_through_neighbor;
        }
      }

      // Add X_new to tree through best "near" node.
      const Node n_new = Node(X_new, best_parent_idx, cost_through_best_parent);
      NodeID n_new_idx = tree.Add(n_new);

      // Connect all neighbors of X_new to X_new if that path cost is less.
      for (const NodeID neighbor_idx : neighbor_idxs) {
        // TODO don't search over best_cost_idx (the best parent for n_new)
        Node n_neighbor = tree.GetNode(neighbor_idx);
        const double neighbor_cost_through_new =
            n_new.cost + DistanceMetric(n_neighbor.position, n_new.position);
        if (neighbor_cost_through_new < n_neighbor.cost and
            !HasCollision(n_new.position, n_neighbor.position)) {
          // Best path for neighbor is now through X_new
          n_neighbor.parent = n_new_idx;
          n_neighbor.cost = neighbor_cost_through_new;
          // Update the neighbor node in the tree.
          tree.SetNode(neighbor_idx, n_neighbor);
        }
      }

      if (AtGoal(n_new.position)) {
        tree.AddSolution(n_new_idx);
      }
    }
  }

  // Print search report.
  tree.Report();

  // return tree or path or something
}

