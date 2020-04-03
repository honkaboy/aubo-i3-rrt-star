#include "tree.h"
#include <assert.h>
#include <iostream>
#include <limits>

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

Tree::Tree(const Node& root,
           std::function<double(const VectorXd&, const VectorXd&)> distance_metric,
           const size_t max_nodes)
    : distance_metric_(distance_metric), kMaxNodes(max_nodes) {
  nodes_.push_back(root);
  // Make root the current best node.
  best_node_and_cost_to_go_ = {0, root.cost_to_go};
}

NodeID Tree::Add(const Node& new_node, bool is_goal) {
  NodeID id_added_node = kNone;
  if (nodes_.size() < kMaxNodes) {
    // Add the node to the tree.
    nodes_.push_back(new_node);
    id_added_node = nodes_.size() - 1;
    // TODO remove this sanity check
    assert(nodes_[id_added_node] == new_node);

    // Conditionally add to goal set.
    if (is_goal) {
      goal_node_idxs_.push_back(id_added_node);
    }

    // Update the best node if this one is better.
    if (new_node.cost_to_go < best_node_and_cost_to_go_.second) {
      best_node_and_cost_to_go_ = {id_added_node, new_node.cost_to_go};
    }
  }
  return id_added_node;
}

std::vector<NodeID> Tree::near_idxs(const VectorXd& position, double radius) {
  std::vector<NodeID> near_nodes;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const double distance = distance_metric_(position, nodes_[i].position);
    if (distance < radius) {
      near_nodes.push_back(i);
    }
  }
  return near_nodes;
}

NodeID Tree::nearest(const VectorXd& position) {
  NodeID nearest = kNone;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const double distance = distance_metric_(position, nodes_[i].position);
    if (distance < nearest_distance) {
      nearest = i;
      nearest_distance = distance;
    }
  }
  return nearest;
}

Node Tree::GetNode(const NodeID node_id) {
  if (node_id >= nodes_.size()) {
    assert(false);
  }
  return nodes_[node_id];
}

VectorXd GetBestNodePosition() {
  return nodes_[best_node_and_cost_to_go_.first].position;
}

void Tree::SetNode(const NodeID node_id, const Node& node) {
  assert(node_id < nodes_.size());
  nodes_[node_id] = node;
  return;
}

void Tree::Report() {
  if (!goal_node_idxs_.empty()) {
    // Find the goal node with the smallest cost (from origin).
    const NodeID best_node_idx =
        *std::min_element(goal_node_idxs_.begin(), goal_node_idxs_.end(),
                          [this](const NodeID a, const NodeID b) {
                            return nodes_[a].cost < nodes_[b].cost;
                          });
    const Node best_node = nodes_[best_node_idx];

    // Print all goal nodes.
    std::cout << "reached goal at nodes: {";
    for (const NodeID node : goal_node_idxs_) {
      std::cout << node << ", ";
    }
    std::cout << std::endl;

    // Look at the best node
    std::vector<NodeID> goal_path;
    NodeID parent = best_node_idx;
    // Backtrack through solution.
    while (parent != kNone) {
      goal_path.push_back(parent);
      parent = GetNode(parent).parent;
    }
    // Print starting at root.
    std::cout << "Goal path: ";
    for (auto it = goal_path.rbegin(); it != goal_path.rend(); ++it) {
      std::cout << "{" << *it << " : " << GetNode(*it).cost << "}, ";
    }
    std::cout << std::endl;
  } else {
    // Find the node that has the smallest cost to go to reach the goal.
    const auto best_node_it = std::min_element(
        nodes_.begin(), nodes_.end(),
        [this](const Node& a, const Node& b) { return a.cost_to_go < b.cost_to_go; });
    const NodeID best_node_idx = std::distance(nodes_.begin(), best_node_it);
    const Node best_node = *best_node_it;
    // TODO remove
    assert(nodes_[best_node_idx] == best_node);

    std::cout << "Goal not reached. Closest node is ";
    std::cout << "{" << best_node_idx << " : cost - " << best_node.cost
              << " : cost_to_go - " << best_node.cost_to_go << "}, " << std::endl;
  }
  return;
}
