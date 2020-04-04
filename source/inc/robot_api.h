#ifndef __robot_api_h__
#define __robot_api_h__

#include <Eigen/Dense>

#include "types.h"
#include "planner_api.h"

class RobotAPI {
 public:
  /// TODO update dox
  /// Inverse Kinematics function returns joint positions required to reach a target pose
  /// given a starting poiot
  /// target_pose - the target pose to obtain corresponding joint positions as an affine
  /// transformation matrix [R [t;1]]
  /// initial_joints - joint positions to start solving from
  /// success - by reference flag where true indicates an ik solution was found returns a
  /// vector of joint positions corresponding to the solution
  static Joint inverse_kinematics(const Pose::AffineTransform_t& target_pose,
                                  const Joint& initial_joints, bool& success);

  // Forward kinematics solution returns the 4d Affine transform matrix [R [t;1]] that
  // corresponds joint positions to a transform
  // joint_target - the vector of joint positions to correspond to a transform returns
  // affine transform matrix
  static Pose::AffineTransform_t forward_kinematics(const Joint& joint_target);

  // Stub collision checking function, where joints is the joint position of the robot to
  // evaluate collisions returns true if collision is detected, false if not
  static bool in_collision(const Joint& joints);
};

#endif
