#ifndef __tree_h__
#define __tree_h__

#include <functional>
#include <vector>
#include "node.h"
#include "types.h"

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

class Tree {
 public:
  Tree(const Node& root,
       std::function<double(const VectorXd&, const VectorXd&)> distance_metric);

  NodeID add(const node& new_node);
  std::vector<NodeID> near_idxs(const VectorXd& position, double radius);
  NodeID nearest(const VectorXd& position);

 private:
  static constexpr uint32_t kMaxNodes = 1000;
  static constexpr NodeID kNone = -1;
  // TODO make const
  const std::function<double(const VectorXd&, const VectorXd&)> distance_metric_;
  std::vector<Node> nodes_;
  std::vector<NodeID> goal_node_idxs_;
};

#endif

/*
class Tree:
  def __init__(self, root, distance_metric):
    self.max_nodes = 10000
    self.nodes = [root]
    self.goal_node_idxs = []
    self.distance_metric = distance_metric

  def add(self, node):
    if len(self.nodes) >= self.max_nodes:
      return None

    self.nodes.append(node)
    idx_added_node = len(self.nodes) - 1

    return idx_added_node

  def near_idxs(self, position, radius):
    near_nodes_idx = []
    near_nodes_ds = []
    for i, node in enumerate(self.nodes):
      this_ds = self.distance_metric(node.position, position)
      if this_ds < radius:
        near_nodes_idx.append(i)
        near_nodes_ds.append(this_ds)

    # Return the near set.
    return near_nodes_idx

  def nearest(self, position):
    nearest_idx = None
    nearest_dist = 10e9
    for i, node in enumerate(self.nodes):
      this_dist = self.distance_metric(node.position, position)
      if this_dist < nearest_dist:
        nearest_idx = i
        nearest_dist = this_dist

    # Return the near set.
    return nearest_idx
*/
