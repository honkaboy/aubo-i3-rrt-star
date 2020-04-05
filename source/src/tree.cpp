#include "tree.h"

#include <iostream>
#include <limits>

constexpr NodeID Tree::kNone;

Tree::Tree(std::function<double(const Joint&, const Joint&)> distance_metric,
           const size_t max_nodes)
    : distance_metric_(distance_metric), kMaxNodes(max_nodes) {
  // So that best-node updating work properly in Tree::Add().
  best_node_and_distance_to_goal_ =
      std::make_pair(kNone, std::numeric_limits<double>::infinity());
}

NodeID Tree::Add(const Node& new_node, bool is_goal) {
  NodeID id_added_node = kNone;
  if (nodes_.size() < kMaxNodes) {
    // Add the node to the tree.
    nodes_.push_back(new_node);
    id_added_node = nodes_.size() - 1;
    if (nodes_[id_added_node] != new_node) {
      throw std::logic_error("ID of added node and the new node don't match.");
    }

    // Conditionally add to goal set.
    if (is_goal) {
      goal_node_idxs_.push_back(id_added_node);
    }

    // Update the best node if this one is better.
    if (new_node.distance_to_goal < best_node_and_distance_to_goal_.second) {
      best_node_and_distance_to_goal_ = {id_added_node, new_node.distance_to_goal};
    }
  }
  return id_added_node;
}

bool Tree::IsFull() const { return nodes_.size() >= kMaxNodes; }

std::vector<NodeID> Tree::near_idxs(const Joint& position, double radius) const {
  std::vector<NodeID> near_nodes;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const double distance = distance_metric_(position, nodes_[i].position);
    if (distance < radius) {
      near_nodes.push_back(i);
    }
  }
  return near_nodes;
}

NodeID Tree::nearest(const Joint& position) const {
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

Node Tree::GetNode(const NodeID node_id) const {
  if (node_id >= nodes_.size()) {
    throw std::runtime_error("Invalid node ID.");
  }
  return nodes_[node_id];
}

Joint Tree::GetBestNodePosition() const {
  return nodes_[best_node_and_distance_to_goal_.first].position;
}

void Tree::SetNode(const NodeID node_id, const Node& node) {
  if (node_id >= nodes_.size()) {
    throw std::runtime_error("Invalid node ID.");
  }
  nodes_[node_id] = node;
  return;
}

std::vector<NodeID> Tree::Solution() const {
  std::vector<NodeID> solution;

  if (!goal_node_idxs_.empty()) {
    // Find the goal node with the smallest cost (from origin).
    const NodeID best_node_idx =
        *std::min_element(goal_node_idxs_.begin(), goal_node_idxs_.end(),
                          [this](const NodeID a, const NodeID b) {
                            return nodes_[a].cost < nodes_[b].cost;
                          });

    // Backtrack from best node to obtain the best solution found.
    NodeID parent = best_node_idx;
    while (parent != kNone) {
      solution.push_back(parent);
      parent = GetNode(parent).parent;
    }
  }
  return solution;
}

void Tree::Report() const {
  if (!goal_node_idxs_.empty()) {
    // Find the goal node with the smallest cost (from origin).
    std::vector<NodeID> solution = Solution();
    const NodeID best_node_idx = solution.back();
    const Node best_node = nodes_[best_node_idx];

    // Print all goal nodes.
    std::cout << "reached goal at nodes: {";
    for (const NodeID node : goal_node_idxs_) {
      std::cout << node << ", ";
    }
    std::cout << std::endl;

    // Print starting at root.
    std::cout << "Solution path: ";
    for (auto it = solution.rbegin(); it != solution.rend(); ++it) {
      std::cout << "{" << *it << " : " << GetNode(*it).cost << "}, ";
    }
    std::cout << std::endl;
  } else {
    // Find the node that has the smallest cost to go to reach the goal.
    const auto best_node_it = std::min_element(
        nodes_.begin(), nodes_.end(), [this](const Node& a, const Node& b) {
          return a.distance_to_goal < b.distance_to_goal;
        });
    const NodeID best_node_idx = std::distance(nodes_.begin(), best_node_it);
    const Node best_node = *best_node_it;
    if (nodes_[best_node_idx] != best_node) {
      throw std::logic_error("Unexpected: Node ID != index.");
    }

    std::cout << "Goal not reached. Closest node is ";
    std::cout << "{" << best_node_idx << " : cost - " << best_node.cost
              << " : distance_to_goal - " << best_node.distance_to_goal << "}, "
              << std::endl;
  }
  return;
}
