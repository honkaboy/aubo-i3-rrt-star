#include "node.h"

Node::Node(const Joint& position, const NodeID parent, const double cost,
           const double distance_to_goal)
    : position(position), parent(parent), cost(cost), distance_to_goal(distance_to_goal) {}
