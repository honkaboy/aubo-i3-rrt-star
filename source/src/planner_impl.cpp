#include "planner_impl.h"

#include "robot_api.h"
#include "tree.h"

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

double PlannerImpl::DistanceMetric(const VectorXd&, const VectorXd&) {
  // TODO
  return 1.0;
}

VectorXd InitialPosition(const Pose& start) {
  VectorXd init(6);
  init << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  return init;
}

// Default definition of a virtual planner
Path PlannerImpl::plan(const Pose& start, const Pose& end, double resolution,
                       bool& plan_ok) {
  /// TODO The robot's initial joint positions should be specified, as supplying simply
  /// the pose to begin likely underconstrains the problem (assuming inverse kinematics
  /// pose -> joints is a many-to-one function.
  VectorXd initial_joints = InitialPosition(start);

  Node root(initial_joints, -1, 0.0);
  Tree tree(root, DistanceMetric);
  plan_ok = false;
  return Path();
}
