#ifndef __node_h__
#define __node_h__

#include <Eigen/Dense>
#include "types.h"

/*
class Node:
  def __init__(self, position, parent, cost):
    self.position = position
    self.parent = parent
    # Cost to get to this node from root.
    self.cost = cost

  def __str__(self):
    return f"parent: {self.parent}; cost: {self.cost}"
*/

struct Node {
  Node(const Eigen::VectorXd& position, const NodeID parent, const double cost);
  inline bool operator==(const Node& right) const {
    return (position == right.position) && (parent == right.parent) &&
           (cost == right.cost);
  }
  Eigen::VectorXd position;
  NodeID parent;
  double cost;
};

#endif
