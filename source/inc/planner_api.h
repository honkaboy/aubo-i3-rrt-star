#ifndef __planner_api_h__
#define __planner_api_h__

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Represents a 3D robot pose consisting of a quaternion and translation vector
// You may add additional methods to this class
class Pose {
 public:
  typedef Eigen::Transform<double, 3, Affine> AffineTransform_t;
  // Express the pose as an affine transform.
  static AffineTransform_t AffineTransform() {
    return Eigen::Translation3d(goal.translation) * goal.orientation_quaternion;
  }
  // The orientation as a quaternion
  Eigen::Quaterniond orientation_quaternion;
  // Translation vector in meters [x, y, z]
  Eigen::Vector3d translation;
};

// Represents a geometric path as a sequential series of joint positions
// You may add additional methods to this class
class Path {
 public:
  // Matrix containing joint positions. Each column represents each joint, each row is a
  // point along the path Matrix shape is thus J columns and S rows, where J is the number
  // of joints in the robot, S is the number of samples
  Eigen::MatrixXd joint_positions;
};

// Pure virtual class representing a path planner with a single "plan" function to plan a
// geometric path
class Planner {
 public:
  // Function all planners will override.
  // start - the starting position of the path
  // end - the ending position of the path
  // resolution - the minimum spacing between incremental path positions (how fine the
  // path is) plan_ok - by reference flag where true means the plan was successfully
  // planned, false means a plan could not be found returns - a Path object representing
  // the planned path
  virtual Path plan(const Pose& start, const Pose& end, double resolution,
                    bool& plan_ok) {
    plan_ok = false;
    return Path();
  }
};

#endif
