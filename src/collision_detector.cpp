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
  else ROS_ERROR("NULL pointer for costmap");
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
  // ROS_INFO("Point (%.4f, %.4f) has cost: %u", wx, wy, cost);
  // cost > 127 means obstacle
  // cost = 0 means totally free space
  if (cost > 0){
    // ROS_INFO("Point at obstacle: (%.4f, %.4f)", wx, wy);
    return true;
  }
  else{
    // ROS_WARN("Chek if this point (%.4f, %.4f) is not in obstacle", wx, wy);
    return false;
  }
}
bool CollisionDetector::isThereObstacleBetween(const Node &node, const std::pair<double, double> &point) {
  // ...

  float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);
  // ROS_INFO("Distance: %.4f", dist);
  
  float cd_resolution=resolution_*2.0; // resolution for collision test

  if (dist < cd_resolution) {
    // ROS_INFO("Distance is less than resolution");
    return (isThisPointCollides(point.first, point.second)) ? true : false;
  } 
  else {
    // check if last point collides
    if (isThisPointCollides(point.first, point.second)){
      // ROS_WARN("Collision detected at goal point. Exiting collision test");
      return true;
    }
    // compute step number
    int steps_number = static_cast<int>(floor(dist/(cd_resolution)));
    // ROS_INFO("Steps number: %d", steps_number);

    float theta = atan2(-node.y + point.second, -node.x + point.first);
    // ROS_INFO("Theta: %.4f", theta);

    std::pair<float, float> p_n;
    for (int n = 1; n < steps_number; n++) {
      p_n.first = node.x + n*cd_resolution*cos(theta);
      p_n.second = node.y + n*cd_resolution*sin(theta);
      // ROS_INFO("Checking point (%.4f, %.4f)", p_n.first, p_n.second);

      if (isThisPointCollides(p_n.first, p_n.second)) {
        // ROS_INFO("Collision detected at point (%.4f, %.4f)", p_n.first, p_n.second);
        return true;
      }
      // ROS_INFO("No collision detected");
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

  float cd_resolution=resolution_*2.0; // resolution for collision test
  
  float dist = euclideanDistance2D(point1.first, point1.second, point2.first, point2.second);
  if (dist < cd_resolution) {
    return (isThisPointCollides(point2.first, point2.second)) ? true : false;
  } 
  else {
    // check if last point collides
    if (isThisPointCollides(point2.first, point2.second)){
      // ROS_WARN("Collision detected at goal point. Exiting collision test");
      return true;
    }
    // compute step number
    int steps_number = static_cast<int>(floor(dist/(cd_resolution)));
    float theta = atan2(-point1.second + point2.second, -point1.first + point2.first);
    std::pair<float, float> p_n;
    for (int n = 1; n < steps_number; n++) {
      p_n.first = point1.first + n*cd_resolution*cos(theta);
      p_n.second = point1.second + n*cd_resolution*sin(theta);
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
    mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  }
}

}  // namespace rrt_star_global_planner
