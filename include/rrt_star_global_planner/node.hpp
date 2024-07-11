/*
  Copyright 2021 - Rafael Barreto
*/

// include guards
#ifndef RRT_STAR_GLOBAL_PLANNER_NODE_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_NODE_HPP_

#include <cmath>

namespace rrt_star_global_planner {
// inline to avoid function overhead, since it will be called a lot of times and it is small
inline float euclideanDistance2D(float x1, float y1, float x2, float y2) {
  return std::hypot((x1 - x2), (y1 - y2));
}

// Node structure
struct Node {
  // variable members
  float x;
  float y;
  int node_id;
  int parent_id;
  float cost{0.0};
  // default constructor
  Node() {}
  /* initializer list
  used to initialize the variable members (calling this constructor) 
  if arguments are provided when an object of this class is created */
  Node(float px, float py, int node_index, int parent_index) 
    : x(px), y(py), node_id(node_index), parent_id(parent_index) {}

  // comparing two Node objects is based on their id 
  bool operator ==(const Node& node) { return node_id == node.node_id; }

  bool operator !=(const Node& node) { return !(node_id == node.node_id); }
};
}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_NODE_HPP_  NOLINT
