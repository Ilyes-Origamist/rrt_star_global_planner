/*
  Copyright 2021 - Rafael Barreto
*/

#include "rrt_star_global_planner/collision_detector.hpp"
#include <ros/ros.h>

namespace rrt_star_global_planner {

CollisionDetector::CollisionDetector(costmap_2d::Costmap2D* costmap) : costmap_(costmap) {
  if (costmap_ != nullptr) {
    resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
  }
}

bool CollisionDetector::isThisPointCollides(float wx, float wy) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    // no collision
    return false;
  }

  int mx, my;
  worldToMap(wx, wy, mx, my);
  // boundary check
  if ((mx < 0) || (my < 0) || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
    return true;

  // getCost returns unsigned char
  unsigned int cost = static_cast<int>(costmap_->getCost(mx, my));
  
  // cost > 127 means obstacle
  // cost = 0 means totally free space
  if (cost > 0)
    return true;
  
  else{
    // ROS_WARN("Chek if this point (%.4f, %.4f) is not in obstacle", wx, wy);
    return false;
  }
}

bool CollisionDetector::isThereObstacleBetween(const Node &node, const std::pair<double, double> &point) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    ROS_ERROR("No Costmap");
    return false;
  }

  float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);
  if (dist < resolution_) {
    return (isThisPointCollides(point.first, point.second)) ? true : false;
  } 
  else {
    int steps_number = static_cast<int>(floor(dist/resolution_));
    float theta = atan2(node.y - point.second, node.x - point.first);
    std::pair<float, float> p_n;
    for (int n = 1; n < steps_number; n++) {
      p_n.first = node.x + n*resolution_*cos(theta);
      p_n.second = node.y + n*resolution_*sin(theta);
      if (isThisPointCollides(p_n.first, p_n.second))
        return true;
    }
    return false;
  }
}


// between two points
bool CollisionDetector::isThereObstacleBetween(const std::pair<double, double> &point1, const std::pair<double, double> &point2) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    // there is NO obstacles
    return false;
  }

  float dist = euclideanDistance2D(point1.first, point1.second, point2.first, point2.second);
  if (dist < resolution_) {
    return (isThisPointCollides(point2.first, point2.second)) ? true : false;
  } 
  else {
    int steps_number = static_cast<int>(floor(dist/resolution_));
    float theta = atan2(point1.second - point2.second, point1.first - point2.first);
    std::pair<float, float> p_n;
    for (int n = 1; n < steps_number; n++) {
      p_n.first = point1.first + n*resolution_*cos(theta);
      p_n.second = point1.second + n*resolution_*sin(theta);
      if (isThisPointCollides(p_n.first, p_n.second))
        return true;
    }
    return false;
  }
}

// between two nodes
bool CollisionDetector::isThereObstacleBetween(const Node &node1, const Node &node2) {
  return isThereObstacleBetween(node1, std::make_pair(node2.x, node2.y));
}


void CollisionDetector::worldToMap(float wx, float wy, int& mx, int& my) {
  if (costmap_ != nullptr) {
    mx = (wx - origin_x_) / resolution_;
    my = (wy - origin_y_) / resolution_;
  }
}

}  // namespace rrt_star_global_planner
