#include <robot_api.h>
#include <rrt_star_planner.h>
#include <iostream>
#include <random>
#include <memory>

int main(int argc, char** argv) {
  // Assuming this is min radian displacement between planned path joint angles.
  const double resolution = 0.01;
  const std::unique_prt<Planner> planner(new RRTStarPlanner(resolution));

  const double min_radius = 0.3;  // m
  const double max_radius = 0.5;  // m

  // Generate "random" to / from poses.
  // NOTE: Using constant seeds here to make the demo of the planning algorithm
  // deterministic (e.g. might generate un-reachable poses otherwise).
  std::default_random_engine generator(1);
  std::uniform_int_distribution<int> distribution(0, 100000);

  const int iterations = 5;
  for (int i = 0; i < iterations; ++i) {
    const Pose pose1(min_radius, max_radius, distribution(generator));
    const Pose pose2(min_radius, max_radius, distribution(generator));

    bool ok;
    Path result = planner->plan(pose1, pose2, resolution, ok);
    if (ok) {
      std::cout << "Path plan was successful" << std::endl;
      std::cout << result.joint_positions << std::endl;
    } else {
      std::cout << "Path plan was unsuccessful" << std::endl;
    }
  }

  return 1;
}

