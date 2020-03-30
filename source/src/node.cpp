#include "node.h"
#include <assert.h>

Node::Node(const Eigen::VectorXd& position, const NodeID parent, const double cost)
    : position(position), parent(parent), cost(cost) {}
