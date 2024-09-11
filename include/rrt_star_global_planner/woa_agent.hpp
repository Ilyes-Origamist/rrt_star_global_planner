/*
  Copyright 2024 - Chaabeni Ilyes
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_WOA_AGENT_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_WOA_AGENT_HPP_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include "rrt_star_global_planner/random_double_generator.hpp"
#include "rrt_star_global_planner/random_int_generator.hpp"
#include "rrt_star_global_planner/collision_detector.hpp"
#include "rrt_star_global_planner/node.hpp"

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>
#include <cstdint>

#include <armadillo>

namespace rrt_star_global_planner {

class PathAgent {
 public:
  PathAgent(std::list<std::pair<float, float>> &path,
            const float sampling_radius,
            uint16_t id,
            costmap_2d::Costmap2D* costmap,
            float spiral_shape);

  //  // Move constructor
  // PathAgent(PathAgent&& other) noexcept;
  // // Move assignment operator
  // PathAgent& operator=(PathAgent&& other) noexcept;
 
  // public data variables 
  float sampling_radius_{0.1};
  costmap_2d::Costmap2D* costmap_{nullptr};
  float map_height_;
  float map_width_;
  bool collides{false};
  

  // the vector representing the path
  arma::vec X;
  // updated in each iteration
  float A{0};
  // updated in each iteration
  float C{0};
  // updated in each iteration
  float l{0};
  uint16_t vec_size;
  float b;
  std::pair<float, float> start_point_;
  std::pair<float, float> goal_point_;
  std::list<std::pair<float, float>> initial_path_; // for display

  /**
   * @brief uses circular search to update the agent Xi 
   * @param search_agent The search agent, either Xbest or Xrand
   */
  void circularUpdate(arma::vec search_agent);

  /**
   * @brief uses spiral search to update the agent Xi 
   * @param search_agent The search agent, which is Xbest
   */
  void spiralUpdate(arma::vec search_agent);

  /**
   * @brief Computes the cost of the agent 
   * @return float representing the length of the path
   */
  float fitness() ;

  /**
   * @brief gets the index / id of the PahtAgent object
   * @return the id (index) of the agent
   */
  uint16_t getID();

  /**
   * @brief generates random initial paths based on the RRT* initial path
   * @return a path close to initial path
   * @param path: the initial path generated by RRT*
   */
  std::list<std::pair<float, float>> randomInitialPath(std::list<std::pair<float, float>> &path);

 private:
  std::list<std::pair<float, float>> path_;
  CollisionDetector collision_;
  arma::vec D;
  arma::vec D2;
  RandomDoubleGenerator random_device_;
  int id_;
  std::pair<float, float> biasedSampling(std::pair<double, double> center);

};

} 

#endif // RRT_STAR_GLOBAL_PLANNER_WOA_AGENT_HPP_  // NOLINT 