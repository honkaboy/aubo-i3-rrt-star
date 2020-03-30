#ifndef __planner_impl_h__
#define __planner_impl_h__

#include <random>

#include <Eigen/Dense>
#include <cmath>
#include "planner_api.h"
#include "types.h"

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double DegreesToRadians(const double degrees) { return degrees / 180.0 * M_PI; }

constexpr double RadiansToDegrees(const double radians) { return radians / M_PI * 180.0; }

class PlannerImpl : public Planner {
 public:
  PlannerImpl(const double precision) : kPrecision(precision) {
    /// Total Euclidean joint "distance" that can be moved by moving each joint
    /// kNodeDegreeJump.
    new_node_distance_ = std::sqrt(kDims * kNodeDegreeJump);
  }

  /// \brief Function all planners will override.
  /// \param[in] start - the starting position of the path
  /// \param[in] end - the ending position of the path
  /// \param[out] plan_ok - by reference flag where true means the plan
  /// was successfully planned, false means a plan could not be found
  /// \return Path object representing the planned path
  Path plan(const Pose& start, const Pose& end, double resolution, bool& plan_ok) final;

  bool HasCollision(const VectorXd& X0, const VectorXd& X1);

  static double DistanceMetric(const VectorXd& X0, const VectorXd& X1);

  bool AtGoal(const VectorXd& position);

  VectorXd RandomX();

  VectorXd Steer(const VectorXd& X_root, const VectorXd& X_goal);

  double CalculateNearRadius();

  void RRT_star(VectorXd X0);

 private:
  // Looks like limits for all joints are [-175, +175] degrees
  // (aubo_i3_kinematics.cpp:632).
  static constexpr double kSymmetricMaxJointAngle = DegreesToRadians(175.0);
  static constexpr double kNodeDegreeJump = DegreesToRadians(5.0);
  static constexpr size_t kDims = 6;
  double new_node_distance_;
  const double kPrecision;
};

#endif

/*
class World:
  def __init__(self):
    self.xrange = [-10, 10]
    self.yrange = [-10, 10]
    self.initial_position = (-8, 0)
    # Refers to Z dimension.
    self.goal_pose_z = 0
    # Distance at which to create new nodes from network.
    self.dq = 1.0
    # Precision of collision checking along paths between nodes.
    self.precision = 0.25

    num_obstacles = 3
    self.obstacles = self.make_obstacles(self.xrange, self.yrange, num_obstacles)

  def plot_obstacles(self, ax):
    for ob in self.obstacles:
      xmin, ymin, xmax, ymax = ob
      ax.plot(np.array([xmin, xmin, xmax, xmax, xmin]),
              np.array([ymin, ymax, ymax, ymin, ymin]), 'red')

  @staticmethod
  def make_obstacles(xrange, yrange, num_obstacles):
    obstacles = []
    for i in range(num_obstacles):
      # Box obstacles
      xmin = random.uniform(*xrange)
      xmax = random.uniform(*xrange)
      if xmax < xmin:
        xmin, xmax = xmax, xmin

      ymin = random.uniform(*yrange)
      ymax = random.uniform(*yrange)
      if ymax < ymin:
        ymin, ymax = ymax, ymin

      obstacle = xmin, ymin, xmax, ymax
      obstacles.append(obstacle)
    return obstacles

  @staticmethod
  def X_distance(position1, position2):
    x1, y1 = position1
    x2, y2 = position2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

  def is_collision(self, position):
    def is_collision_obstacle(position, obstacle):
      x, y = position
      xmin, ymin, xmax, ymax = obstacle
      if xmin <= x <= xmax and ymin <= y <= ymax:
        return True
      return False

    for obstacle in self.obstacles:
      if is_collision_obstacle(position, obstacle):
        return True
    return False

  def has_collision(self, X0, X1):
    if X0 == X1:
      return self.is_collision(X0)

    # Number of intermediate points, including origin.
    path_count = math.ceil(self.X_distance(X0, X1) / self.precision)
    x0, y0 = X0
    x1, y1 = X1
    dx = (x1 - x0) / path_count
    dy = (y1 - y0) / path_count

    for i in range(path_count):
      xi = x0 + i * dx
      yi = y0 + i * dy
      Xi = (xi, yi)
      if self.is_collision(Xi):
        return True
    if self.is_collision(X1):
      return True
    return False

  def at_goal(self, position):
    z_resolution = 0.1
    if abs(self.z(position) - self.goal_pose_z) < z_resolution:
      return True
    else:
      return False

  # In our state space, the robot has DOFs X, Y, and "pose" / output state z = f(X) =
f(x,y) def z(self, position): g_x, g_y = 7.5, 0 x, y = position z = (x - g_x)**2 + (y -
g_y)**2 return z

  def random_X(self):
    return (random.uniform(*self.xrange), random.uniform(*self.yrange))

  def steer(self, root_position, goal_position):
    d = self.X_distance(root_position, goal_position)
    dt = self.dq / d  # parametric distance from root to goal

    x_root, y_root = root_position
    x_goal, y_goal = goal_position

    dx = (x_goal - x_root) * dt
    dy = (y_goal - y_root) * dt

    x_out = x_root + dx
    y_out = y_root + dy
    return x_out, y_out

  def rrt_star(self):
    root = Node(self.initial_position, None, 0)
    distance_metric = self.X_distance
    tree = Tree(root, distance_metric)

    # TODO Handle case where root is already at goal.
    # TODO Handle case where root is in collision.
    # TODO Handle case where goal is in collision.

    fig, ax = plt.subplots()

    max_expansion = 1000
    for i in range(max_expansion):
      # TODO Add occasional greedy choice.
      X_random = self.random_X()
      nearest_node_idx = tree.nearest(X_random)
      X_nearest = tree.nodes[nearest_node_idx].position
      X_new = self.steer(X_nearest, X_random)


      # Add to node list if it's not in collision.
      if not self.has_collision(X_nearest, X_new):
        # Note: This does not yet contain X_new
        neighbor_idxs = tree.near_idxs(position=X_new, radius=2.0)
        # Connect X_new to best "near" node. Cost to traverse is euclidean distance in X.
        n_nearest = tree.nodes[nearest_node_idx]
        best_parent_idx = nearest_node_idx
        # Minimum cost to get to X_new through neighbors.
        cost_through_best_parent = n_nearest.cost + self.X_distance(n_nearest.position,
X_new) for neighbor_idx in neighbor_idxs: n_neighbor = tree.nodes[neighbor_idx]
          new_cost_through_neighbor = n_neighbor.cost +
self.X_distance(n_neighbor.position, X_new) if new_cost_through_neighbor <
cost_through_best_parent and not self.has_collision( n_neighbor.position, X_new):
            best_parent_idx = neighbor_idx
            cost_through_best_parent = new_cost_through_neighbor

        # Add X_new to tree through best "near" node.
        n_new = Node(X_new, best_parent_idx, cost_through_best_parent)
        # print("added node")
        # print(n_new)
        n_new_idx = tree.add(n_new)

        # Connect all neighbors of X_new to X_new if that path cost is less.
        for neighbor_idx in neighbor_idxs:
          # TODO don't search over best_cost_idx (the best parent for n_new)
          n_neighbor = tree.nodes[neighbor_idx]
          neighbor_cost_through_new = n_new.cost + self.X_distance(n_neighbor.position,
                                                                   n_new.position)
          if neighbor_cost_through_new < n_neighbor.cost and not self.has_collision(
                  n_new.position, n_neighbor.position):
            # Best path for neighbor is now through x_new
            n_neighbor.parent = n_new_idx
            n_neighbor.cost = neighbor_cost_through_new
            # Update the neighbor node in the tree.
            tree.nodes[neighbor_idx] = n_neighbor

        # If the new node is at the goal, so noted.
        if self.at_goal(n_new.position):
          tree.goal_node_idxs.append(n_new_idx)

        # Plot
        ax.scatter(X_new[0], X_new[1], c='black')
        # ax.annotate(str(n_new_idx), (X_new[0] + 0.1, X_new[1] + 0.1), c='blue')
        X_parent = tree.nodes[n_new.parent].position
        ax.plot(np.array([X_parent[0], X_new[0]]), np.array([X_parent[1], X_new[1]]),
'black')

    if tree.goal_node_idxs:
      print("reached goal at nodes:", tree.goal_node_idxs)
      goal_node_idx = tree.goal_node_idxs[0]
      goal_path = []
      parent = goal_node_idx
      while parent:
        goal_path.append(parent)
        parent = tree.nodes[parent].parent
      # Print starting at root.
      goal_path = list(reversed(goal_path))
      print(f"path through {goal_node_idx}: {goal_path}")
      print("cost:", tree.nodes[goal_node_idx].cost)

    self.plot_obstacles(ax)
*/

