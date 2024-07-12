/*
  Copyright 2021 - Rafael Barreto
*/

#include <pluginlib/class_list_macros.h>

#include "rrt_star_global_planner/rrt_star_planner.hpp"
#include <chrono>
// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_star_global_planner::RRTStarPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_star_global_planner {

// ---------------------------------
// Initialization of the planner
// ---------------------------------

// Initializer list n0
RRTStarPlanner::RRTStarPlanner() : costmap_(nullptr), initialized_(false) {}
// Initializer list n1
RRTStarPlanner::RRTStarPlanner(std::string name,
                               costmap_2d::Costmap2DROS* costmap_ros) : costmap_(nullptr), initialized_(false) {
  // initialize the planner
  initialize(name, costmap_ros);
}

// Initializer list n2
RRTStarPlanner::RRTStarPlanner(std::string name,
                               costmap_2d::Costmap2D* costmap,
                               std::string global_frame) : costmap_(nullptr), initialized_(false) {
  // initialize the planner
  initialize(name, costmap, global_frame);
}

// ---------------------------------
//     Initialization Functions
// ---------------------------------

// Initializer list n1 function definition
void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

// Initializer list n2 function definition
// Initializes the parameters
void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) {
  if (!initialized_) {

    costmap_ = costmap;
    global_frame_ = global_frame;

    ros::NodeHandle private_nh("~/" + name);
    // initialize path publisher
    path_pub_ = private_nh.advertise<nav_msgs::Path>("/move_base/RRTStarPlanner/global_plan", 1, true);
    private_nh.param("goal_tolerance", goal_tolerance_, 0.2);
    private_nh.param("radius", radius_, 0.5);
    private_nh.param("epsilon", epsilon_, 0.1);
    private_nh.param("max_num_nodes", max_num_nodes_, 5000);
    private_nh.param("min_num_nodes", min_num_nodes_, 500);
    private_nh.param("sampling_radius", sampling_radius_, 0.03);

    // TODO(Rafael) remove hard coding
    if (search_specific_area_) {
      map_width_ = 10.0;
      map_height_ = 10.0;
    } else {
      map_width_ = costmap_->getSizeInMetersX();
      map_height_ = costmap_->getSizeInMetersY();
    }

    ROS_INFO("RRT* Global Planner initialized successfully.");
    ROS_INFO("Map dimensions: %f m, %f m", map_width_, map_height_);
    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized... doing nothing.");
  }
}

// ---------------------------------
//     makePlan Function
// ---------------------------------


void RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan) {
  // clear the plan, just in case
  plan.clear();

  ROS_INFO("RRT* Global Planner");
  ROS_INFO("Current Position: ( %.2lf, %.2lf)", start.pose.position.x, start.pose.position.y);
  ROS_INFO("GOAL Position: ( %.2lf, %.2lf)", goal.pose.position.x, goal.pose.position.y);

  std::pair<float, float> start_point = {start.pose.position.x, start.pose.position.y};
  std::pair<float, float> goal_point = {goal.pose.position.x, goal.pose.position.y};

  planner_ = std::shared_ptr<RRTStar>(new RRTStar(start_point,
                                                  goal_point,
                                                  costmap_,
                                                  goal_tolerance_,
                                                  radius_,
                                                  epsilon_,
                                                  max_num_nodes_,
                                                  min_num_nodes_,
                                                  sampling_radius_,
                                                  map_width_,
                                                  map_height_));

  std::list<std::pair<float, float>> path;

  if (planner_->initialPath(path)) {
    ROS_INFO("RRT* Global Planner: Initial Path found!");
    computeFinalPlan(plan, path);
    planner_->optimizePath(path);
    computeFinalPlan(plan, path);
  } else {
    ROS_WARN("The planner failed to find a path, choose other goal position");
  }
}

// ---------------------------------
//     ComputeFinalPlan Function
// ---------------------------------

void  RRTStarPlanner::computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,
                                       const std::list<std::pair<float, float>> &path) {
  // clean plan
  plan.clear();
  auto start_time = std::chrono::high_resolution_clock::now();
  ros::Time plan_time = ros::Time::now();

  // convert points to poses
  for (const auto &point : path) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }
  // Publish the path
  nav_msgs::Path path_msg;
  path_msg.header.stamp = plan_time;
  path_msg.header.frame_id = global_frame_;
  path_msg.poses = plan;
  path_pub_.publish(path_msg);
  ROS_INFO("Published path with %ld points.", plan.size());
  ROS_INFO("Path Length is: %ld", path.size());

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ROS_INFO("Time taken by initialPath: %f seconds", diff.count());
}

}  // namespace rrt_star_global_planner
