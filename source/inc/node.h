#ifndef __node_h__
#define __node_h__

#include <Eigen/Dense>
#include "types.h"

struct Node {
  Node(const Joint& position, const NodeID parent, const double cost,
       const double cost_to_go);
  inline bool operator==(const Node& right) const {
    return (position == right.position) && (parent == right.parent) &&
           (cost == right.cost) && (cost_to_go == right.cost_to_go);
  }
  Joint position;
  NodeID parent;
  double cost;
  double cost_to_go;
};

#endif
