#ifndef __node_h__
#define __node_h__

#include <Eigen/Dense>
#include "types.h"

struct Node {
  Node(const Joint& position, const NodeID parent, const double cost,
       const double distance_to_goal);
  inline bool operator==(const Node& right) const {
    return (position == right.position) && (parent == right.parent) &&
           (cost == right.cost) && (distance_to_goal == right.distance_to_goal);
  }
  inline bool operator!=(const Node& right) const { return !(*this == right); }

  // 6DOF joint displacements.
  Joint position;

  // This node's optimal predecessor.
  NodeID parent;

  // The cost to get to this node.
  double cost;

  // A measure of the distance to the goal.
  double distance_to_goal;
};

#endif
