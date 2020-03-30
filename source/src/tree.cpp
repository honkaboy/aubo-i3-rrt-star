#include "tree.h"

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

Tree::Tree(const Node& root,
           std::function<double(const VectorXd&, const VectorXd&)> distance_metric)
    : distance_metric(distance_metric) {
  nodes.push_back(root);
}
