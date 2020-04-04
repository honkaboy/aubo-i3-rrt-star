#include <planner_impl.h>
#include <robot_api.h>
#include <iostream>

int main(int argc, char** argv) {
  // Assuming this is min radian displacement between planned path joint angles.
  const double resolution = 0.01;
  Planner* planner;
  // Replace with a new version of your planner here
  planner = new PlannerImpl(resolution);

  const double min_radius = 0.3;  // m? Don't know how aubo i3 units are defined
  const double max_radius = 0.5;

  // Generate "random" to / from poses.
  // NOTE: Using constant seeds here to make the demo of the planning algorithm
  // deterministic (e.g. might generate un-reachable poses otherwise).
  const unsigned seed1 = 2100272238;
  const unsigned seed2 = 2100390024;
  const Pose pose1(min_radius, max_radius, seed1);
  const Pose pose2(min_radius, max_radius, seed2);

  bool ok;
  Path result = planner->plan(pose1, pose2, resolution, ok);
  if (ok) {
    std::cout << "Path plan was successful" << std::endl;
  } else {
    std::cout << "Path plan was unsuccessful" << std::endl;
  }

  return 1;
}

