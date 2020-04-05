#ifndef __planner_api_h__
#define __planner_api_h__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cassert>
#include <chrono>
#include <cmath>
#include <random>

// Represents a 3D robot pose consisting of a quaternion and translation vector
class Pose {
 public:
  typedef Eigen::Transform<double, 3, Eigen::Affine> AffineTransform_t;
  // Express the pose as an affine transform.
  AffineTransform_t AffineTransform() const {
    return Eigen::Translation3d(translation) * orientation_quaternion;
  }
  Pose() {}
  Pose(const double min_radius, const double max_radius, const unsigned seed) {
    // TODO better error handling than this.
    assert(max_radius >= min_radius);

    // TODO not the most efficient to recreate the engine every call, maybe? But this
    // function isn't called too many times, so not worth optimizing right now for the
    // sake of time.
    // Note: Option to generate poses randomly here every time.
    const unsigned seed2 = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 gen(seed2 + seed);
    std::uniform_real_distribution<double> dist(0, 1);

    // Generate random scale [0,1]
    const double scale = dist(gen) * (max_radius - min_radius) + min_radius;

    // Make random rotation.
    // Random angle [-pi, pi]
    const double angle = dist(gen) * 2 * M_PI - M_PI;
    const Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();

    // Make random pose with radius <= \p radius.
    translation = scale * Eigen::Vector3d::Random().normalized();
    orientation_quaternion = Eigen::AngleAxisd(angle, axis);
  }
  // The orientation as a quaternion
  Eigen::Quaterniond orientation_quaternion;
  // Translation vector in meters [x, y, z]
  Eigen::Vector3d translation;
};

// Represents a geometric path as a sequential series of joint positions
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
