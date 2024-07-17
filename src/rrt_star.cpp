/*
  Copyright 2021 - Rafael Barreto
*/

#include "rrt_star_global_planner/rrt_star.hpp"
#include <chrono>


namespace rrt_star_global_planner {

// Constructor
// Initializer list
RRTStar::RRTStar(const std::pair<float, float> &start_point,
                 const std::pair<float, float> &goal_point,
                 costmap_2d::Costmap2D* costmap,
                 double goal_tolerance,
                 double radius,
                 double epsilon,
                 unsigned int max_num_nodes,
                 unsigned int min_num_nodes,
                 double sampling_radius,
                 float map_width,
                 float map_height) : start_point_(start_point),
                                     goal_point_(goal_point),
                                     costmap_(costmap),
                                     goal_tolerance_(goal_tolerance),
                                     radius_(radius),
                                     epsilon_(epsilon),
                                     max_num_nodes_(max_num_nodes),
                                     min_num_nodes_(min_num_nodes),
                                     sampling_radius_(sampling_radius),
                                     map_width_(map_width),
                                     map_height_(map_height),
                                     cd_(costmap) {
  nodes_.reserve(max_num_nodes_);

  // Set default range
  random_double_.setRange(-map_width_, map_width_);
}

/*
finding initial path
*/
bool RRTStar::initialPath(std::list<std::pair<float, float>> &path) {
  goal_reached_ = false;

  if (cd_.isThisPointCollides(goal_point_.first, goal_point_.second)) {
    ROS_ERROR("Goal point chosen is NOT in the FREE SPACE! Choose other goal!");
    return false;
  }

  // Start Node
  createNewNode(start_point_.first, start_point_.second, -1);

  std::pair<float, float> p_rand;
  std::pair<float, float> p_new;

  Node node_nearest;

  bool found_next;

  auto start_time = std::chrono::high_resolution_clock::now();

  // main loop
  while (nodes_.size() < max_num_nodes_ && !goal_reached_) {
    found_next = false;

    while (!found_next) {
      /* continue generating a random point until it is possible 
      without any collision */
      p_rand = sampleFree();  // random point in the free space
      node_nearest = nodes_[getNearestNodeId(p_rand)];  // nearest node of the random point
      p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);  // new point and node candidate
      if (!cd_.isThereObstacleBetween(node_nearest, p_new)) {
        found_next = true;
        createNewNode(p_new.first, p_new.second, node_nearest.node_id);
      }
    }
    // after p_new is generated, check if it is within goal's vicinity
    goal_reached_=isGoalReached(p_new);
  }

  // goal reached: initial path found
  if(goal_reached_){
    goal_node_ = nodes_.back();
    computeFinalPath(path);
    ROS_INFO("Initial Path found! Proceeding to path optimization.");
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    ROS_INFO("Time taken to find initia path: %f seconds", diff.count());
    return true;
  }
  // max number of nodes reached
  else{
    max_nodes_reached=true;
    ROS_ERROR("Max number of nodes reached! Did not find a path!");
    return false;
  }
}


/*
optimizing initial path
*/
void RRTStar::optimizePath(std::list<std::pair<float, float>> &path) {

  std::pair<float, float> p_rand;
  std::pair<float, float> p_new;
  // std::pair<float, float> current_point;
  Node node_nearest;

  bool found_next=false;
  int num_travels_=0;
  auto it = path.begin();  // Iterator to traverse the list
  // main loop
  while (nodes_.size() < max_num_nodes_) {
    // if travelled all the path (except goal node)
    if (it == std::prev(path.end())) {
      it = path.begin();
      num_travels_++;
      computeFinalPath(path);
      ROS_INFO("Traveled the path for %d time", num_travels_);
    }
    else {
      found_next = false; // Reset found_next before the inner loop
      while (!found_next) {
        // current node = path[i] when traveling
        // biased sampling with current node as center of the circle
        if (it == path.end() ||(it->first > 500) || (it->second > 500)) {
          ROS_ERROR("Iterator 'it' is invalid.");
          return;
        }        
        p_rand= biasedSampling(*it);
        if (nodes_.empty()) {
          ROS_ERROR("Nodes vector is empty.");
          return;
        }        
        node_nearest = nodes_[getNearestNodeId(p_rand)];  // nearest node of the random point
        p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);  // new point and node candidate
        if (!cd_.isThereObstacleBetween(node_nearest, p_new)) {
          found_next = true;
          createNewNode(p_new.first, p_new.second, node_nearest.node_id);
        }
      }
      // moving to next node
      ROS_INFO("Current point: (%f,%f)", it->first, it->second);
      ++it;
    }
  }
  // end of optimization loop, recompute the final path
  computeFinalPath(path);
  // end
}

/*
sampleFree 
*/
std::pair<float, float> RRTStar::sampleFree() {
  std::pair<float, float> random_point;
  random_point.first = random_double_.generate();
  random_point.second = random_double_.generate();

  return random_point;
}


/*
biasedSampling
*/
std::pair<float, float> RRTStar::biasedSampling(std::pair<double, double> center) {
  std::pair<float, float> new_rand_point;  
  double min_x = center.first - sampling_radius_;
  double max_x = center.first + sampling_radius_;
  double min_y = center.second - sampling_radius_;
  double max_y = center.second + sampling_radius_;
  random_double_.setRange(min_x,max_x);
  new_rand_point.first = random_double_.generate();
  random_double_.setRange(min_y,max_y);
  new_rand_point.second = random_double_.generate();
  
  return new_rand_point;
}

/*
getNearestNodeId
*/
int RRTStar::getNearestNodeId(const std::pair<float, float> &point) {
  float dist_nearest, dist;
  Node node_nearest = nodes_[0];
  for (int i = 1; i < nodes_.size(); ++i) {
    dist_nearest = euclideanDistance2D(node_nearest.x, node_nearest.y, point.first, point.second);
    dist = euclideanDistance2D(nodes_[i].x, nodes_[i].y, point.first, point.second);
    if (dist < dist_nearest) node_nearest = nodes_[i];
  }

  return node_nearest.node_id;
}

/*
createNewNode
*/
void RRTStar::createNewNode(float x, float y, int node_nearest_id) {
  // new node placed using steer
  Node new_node(x, y, node_count_, node_nearest_id);
  nodes_.emplace_back(new_node);

  if (node_nearest_id != -1) {
    // Optimize
    chooseParent(node_nearest_id);
    rewire();
  }
  node_count_++;
}

/*
chooseParent
*/
void RRTStar::chooseParent(int node_nearest_id) {
  float cost_new_node;
  float cost_other_parent;
  float nodes_dist;
  // parent node initialized to nearest node
  Node parent_node = nodes_[node_nearest_id];

  Node &new_node = nodes_.back();

  for (const auto &node : nodes_) {
    if (node.node_id == new_node.node_id) continue;
    // distance between node and new_node
    nodes_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);

    if (nodes_dist < radius_) {
      // current cost of new_node
      cost_new_node = parent_node.cost + euclideanDistance2D(parent_node.x, parent_node.y, new_node.x, new_node.y);

      // cost if the parent is node
      cost_other_parent = node.cost + nodes_dist;

      if (cost_other_parent < cost_new_node) {
        if (!cd_.isThereObstacleBetween(node, new_node)) {
          parent_node = node;
        }
      }
    }
  }

  // Update new_node cost and its new parent
  new_node.cost = parent_node.cost + euclideanDistance2D(parent_node.x, parent_node.y, new_node.x, new_node.y);
  new_node.parent_id = parent_node.node_id;
}

/*
rewire
*/
void RRTStar::rewire() {
  float nodes_dist;
  float cost_node;

  Node new_node = nodes_.back();

  for (auto &node : nodes_) {
    // distance between node and new_node
    nodes_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);

    // check if node is already the parent and if node is near for optimization
    if (node != nodes_[new_node.parent_id] && nodes_dist < radius_) {
      // cost if the parent of node is new_node
      cost_node = new_node.cost + euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);

      if (cost_node < node.cost && !cd_.isThereObstacleBetween(node, new_node)) {
        // update the new parent of node and its new cost
        node.parent_id = new_node.node_id;
        node.cost = cost_node;
      }
    }
  }
}

/*
steer
*/
std::pair<float, float> RRTStar::steer(float x1, float y1, float x2, float y2) {
  std::pair<float, float> p_new;
  float dist = euclideanDistance2D(x1, y1, x2, y2);
  if (dist < epsilon_) {
    p_new.first = x1;
    p_new.second = y1;
    return p_new;
  } else {
    float theta = atan2(y2 - y1, x2 - x1);
    p_new.first = x1 + epsilon_*cos(theta);
    p_new.second = y1 + epsilon_*sin(theta);
    return p_new;
  }
}

/*
getNodes
*/
std::vector<Node> RRTStar::getNodes() const {
  return nodes_;
}

/*
computeFinalPath
*/
void RRTStar::computeFinalPath(std::list<std::pair<float, float>> &path) {
  path.clear();

  // Compute the path from the goal to the start
  Node current_node = goal_node_;

  // Final Path
  std::pair<float, float> point;

  do {
    point.first = current_node.x;
    point.second = current_node.y;
    path.push_front(point);

    if (current_node.parent_id < 0 || current_node.parent_id >= nodes_.size()) {
      ROS_ERROR("Invalid parent_id: %d", current_node.parent_id);
      break;
    }

    // update the current node
    current_node = nodes_[current_node.parent_id];
  } while (current_node.parent_id != -1);

  ROS_INFO("Path cost: %f", goal_node_.cost);
}

/*
isGoalReached
*/
bool RRTStar::isGoalReached(const std::pair<float, float> &p_new) {
  return (euclideanDistance2D(p_new.first,
                              p_new.second,
                              goal_point_.first,
                              goal_point_.second) < goal_tolerance_) ? true : false;
}

} 