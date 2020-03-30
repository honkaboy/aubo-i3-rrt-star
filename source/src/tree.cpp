#include "tree.h"
#include <assert>
#include <limits>
#include <iostream>

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

Tree::Tree(const Node& root,
           function<double(const VectorXd&, const VectorXd&)> distance_metric)
    : distance_metric(distance_metric) {
  nodes_.push_back(root);
}

NodeID Tree::add(const node& new_node) {
  NodeID node_id = kNone;
  if (nodes_.size() >= kMaxNodes) {
    node_id = kNone;
  } else {
    nodes_.push_back(new_node);
    idx_added_node = nodes_.size() - 1;
    // TODO remove
    assert(nodes_[idx_added_node] == new_node);
    return idx_added_node;
  }
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
  double nearest_distance = std::numerical_limits<double>::infinity();
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
  assert(node_id < nodes_.size());
  return nodes_[node_id];
}

void Tree::SetNode(const NodeID node_id, const Node& node) {
  assert(node_id < nodes_.size());
  nodes_[node_id] = node;
  return;
}

void Tree::AddSolution(const NodeID node_id) {
  assert(node_id < nodes_.size());
  goal_node_idxs_.push_back(node_id);
}

void Tree:report() {
  if (!goal_node_idxs_.empty()) {
    std::cout << "reached goal at nodes:" << goal_node_idxs_ << std::endl;
    // TODO search for best instead of just using first.
    const NodeID goal_node_idx = goal_node_idxs_[0];
    std::Vector<NodeID> goal_path;
    NodeID parent = goal_node_idx;
    // Backtrack through solution.
    while (parent != kNone) {
      goal_path.push_back(parent);
      parent = GetNode(parent).parent;
    }
    // Print starting at root.
    for (size_t i = goal_path.size() - 1; i > 0; ++i) {
      std::cout << "Path through " << goal_node_idx << ": " << goal_path << std::endl;
      std::cout << "Cost: " << GetNode(goal_node_idx).cost << std::endl;
    }
  }
}
