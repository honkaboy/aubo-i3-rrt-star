#include <planner_impl.h>
#include <robot_api.h>
#include <iostream>

int main(int argc, char** argv) {
  // Assuming this is min radian displacement between planned path joint angles.
  const double resolution = 0.01;
  Planner* planner;
  // Replace with a new version of your planner here
  planner = new PlannerImpl(resolution);
  // basic test case format
  // make 2 poses
  // You will find and fill valid poses
  Pose pose1, pose2;
  // make response variable
  bool ok;
  // call planner
  Path result = planner->plan(pose1, pose2, resolution, ok);
  if (ok) {
    std::cout << "Path plan was successful" << std::endl;
  } else {
    std::cout << "Path plan was unsuccessful" << std::endl;
  }

  return 1;
}

