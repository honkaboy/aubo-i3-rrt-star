#include "tree.h"
#include <limits>

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
