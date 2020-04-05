#ifndef __tree_h__
#define __tree_h__

#include <functional>
#include <limits>
#include <vector>
#include "node.h"
#include "types.h"

class Tree {
 public:
  Tree(std::function<double(const Joint&, const Joint&)> distance_metric,
       const size_t max_nodes);

  static constexpr NodeID kNone = -1;

  // Add a node to the tree.
  NodeID Add(const Node& new_node, bool is_goal = false);

  // Returns true if the tree is its predefined capacity.
  bool IsFull() const;

  // Find nodes in the tree at most \p radius from \position according to
  // distance_metric_.
  std::vector<NodeID> near_idxs(const Joint& position, double radius) const;

  // Find the nearest node to \position according to distance_metric_.
  NodeID nearest(const Joint& position) const;

  // Get the location of the current "best" node (according to distance_to_goal).
  Joint GetBestNodePosition() const;

  // Node accessor.
  Node GetNode(const NodeID node_id) const;

  // Node setter.
  void SetNode(const NodeID node_id, const Node& node);

  // Returns true if a solution path exists in the tree.
  bool HasSolution() const { return !goal_node_idxs_.empty(); }

  // Return the solution, if any exists, with the lowest cost.
  std::vector<NodeID> Solution() const;

  // Print a debug report about the tree.
  void Report() const;

 private:
  // A metric to measure distances between nodes.
  const std::function<double(const Joint&, const Joint&)> distance_metric_;

  // The maximum node storage allowed for the tree.
  const size_t kMaxNodes;

  // Stores tree node data.
  std::vector<Node> nodes_;

  // The IDs of nodes that have reached the goal.
  std::vector<NodeID> goal_node_idxs_;

  // The best node found so far and a distance to goal (agnostic to distance metric).
  std::pair<NodeID, double> best_node_and_distance_to_goal_;
};

#endif
