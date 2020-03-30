#include <robot_api.h>
#include "aubo_i3_kinematics.h"
#include "types.h"

Eigen::VectorXd inverse_kinematics(const Eigen::Matrix4d target_pose,
                                   const Eigen::VectorXd& initial_joints, bool& success) {
  Eigen::VectorXd result(6);
  success = GetInverseResult(target_pose, initial_joints, result);
  return result;
}

Eigen::MatrixXd forward_kinematics(const Eigen::VectorXd& joint_target) {
  Eigen::MatrixXd result(4, 4);
  aubo_forward(result, joint_target);
  return result;
}

bool in_collision(const Eigen::VectorXd& joints) { return false; }
