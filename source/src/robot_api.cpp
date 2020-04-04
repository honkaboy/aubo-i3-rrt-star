#include "robot_api.h"
#include "aubo_i3_kinematics.h"

Joint RobotAPI::inverse_kinematics(const Pose::AffineTransform_t& target_pose,
                                   const Joint& initial_joints, bool& success) {
  Eigen::VectorXd result(6);
  success = GetInverseResult(target_pose.matrix(), initial_joints, result);
  return result;
}

Pose::AffineTransform_t RobotAPI::forward_kinematics(const Joint& joint_target) {
  Eigen::MatrixXd result(4, 4);
  aubo_forward(result, joint_target);
  Pose::AffineTransform_t result_T;
  result_T.matrix() = result;
  return result_T;
}

bool RobotAPI::in_collision(const Joint& joints) {
  // TODO
  return false;
}
