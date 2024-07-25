/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>
#include <memory>

#include "rrt_star_global_planner/node.hpp"
#include "rrt_star_global_planner/rrt_star.hpp"
#include "rrt_star_global_planner/random_double_generator.hpp"
#include "rrt_star_global_planner/random_int_generator.hpp"


namespace rrt_star_global_planner {

/**
 * @class RRTStarPlanner
 * @brief Provides a ROS rrt* global planner plugin
 */
class RRTStarPlanner : public nav_core::BaseGlobalPlanner {
 public:
  RRTStarPlanner();

  /**
   * @brief  Constructor for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use
   */
  RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Constructor for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use
   * @param  global_frame The global frame of the costmap
   */
  RRTStarPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief  Initialization function for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use for planning
   * @param  global_frame The global frame of the costmap
   */
  void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief Given a start and goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);  // NOLINT

  void computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,  // NOLINT
                        const std::list<std::pair<float, float>> &path);

  void computeInitialPlan(std::vector<geometry_msgs::PoseStamped>& plan,  // NOLINT
                        const std::list<std::pair<float, float>> &path);

  void woaOptimizePath(std::list<std::pair<float, float>> &path, int N, int Ng, float spiral_shape);

 private:
  ros::Publisher path_pub_;  
  ros::Publisher initial_path_pub_;

  costmap_2d::Costmap2D* costmap_{nullptr};
  bool initialized_{false};
  int max_num_nodes_;
  int min_num_nodes_;
  double epsilon_;
  float map_width_;
  float map_height_;
  // parameters for RRT*
  double radius_;
  double goal_tolerance_;
  double sampling_radius_;
  bool search_specific_area_{false};
  std::string global_frame_;
  std::shared_ptr<RRTStar> planner_;
  // parameters for WOA
  int N_; // max number of iterations
  int Ng_; // number of agents for WOA
  float b_; // spiral shaping parameter
  // tandom devices for WOA
  RandomDoubleGenerator r_rand, p_rand, l_rand;
  RandomIntGenerator rand_index;
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_  // NOLINT
