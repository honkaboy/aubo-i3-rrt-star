#include "node.h"

Node::Node(const Joint& position, const NodeID parent, const double cost,
           const double cost_to_go)
    : position(position), parent(parent), cost(cost), cost_to_go(cost_to_go) {}
