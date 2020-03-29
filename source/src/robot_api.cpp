#include <robot_api.h>





//Inverse Kinematics function returns joint positions required to reach a target pose given a starting point
//target_pose - the target pose to obtain corresponding joint positions as an affine transformation matrix [R [t;1]]
//initial_joints - joint positions to start solving from
//success - by reference flag where true indicates an ik solution was found
//returns a vector of joint positions corresponding to the 
Eigen::VectorXd inverse_kinematics(const Eigen::Matrix4d target_pose, const Eigen::VectorXd& initial_joints, bool& success)
{
    Eigen::VectorXd result(6);
    success = GetInverseResult(target_pose, initial_joints, result);
    return result;
}



//Forward kinematics solution returns the 4d Affine transform matrix [R [t;1]] that corresponds joint positions to a transform
//joint_target - the vector of joint positions to correspond to a transform
//returns affine transform matrix
Eigen::MatrixXd forward_kinematics(const Eigen::VectorXd& joint_target)
{
    Eigen::MatrixXd result(4,4);
    aubo_forward(result, joint_target);
    return result;
}

//Stub collision checking function, where joints is the joint position of the robot to evaluate collisions
//Returs true if collision is detected, false if not
bool in_collision(const Eigen::VectorXd& joints)
{
    return false;
}
