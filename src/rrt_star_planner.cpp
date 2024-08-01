/*
  Copyright 2021 - Rafael Barreto
*/

#include <pluginlib/class_list_macros.h>

#include "rrt_star_global_planner/rrt_star_planner.hpp"
#include "rrt_star_global_planner/woa_agent.hpp"
#include <chrono>
#include <cmath>
#include <armadillo>

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
    initial_path_pub_ = private_nh.advertise<nav_msgs::Path>("/move_base/RRTStarPlanner/initial_plan", 1, true);
    // RRT Star Parameters
    private_nh.param("goal_tolerance", goal_tolerance_, 0.2);
    private_nh.param("radius", radius_, 0.5);
    private_nh.param("epsilon", epsilon_, 0.1);
    private_nh.param("max_num_nodes", max_num_nodes_, 5000);
    private_nh.param("min_num_nodes", min_num_nodes_, 500);
    
    // WOA parameters
    // sampling_radius_: to generate random initial paths (initialize WOA)
    private_nh.param("sampling_radius",sampling_radius_ , 0.1); 
    private_nh.param("max_iterations", N_, 250);
    private_nh.param("num_agents", Ng_, 10);
    private_nh.param("spiral_shape", b_, 0.5f);

    // TODO(Rafael) remove hard coding
    if (search_specific_area_) {
      map_width_ = 10.0;
      map_height_ = 10.0;
    } else {
      map_width_ = costmap_->getSizeInMetersX();
      map_height_ = costmap_->getSizeInMetersY();
    }

    // initialize random objects (random devices)
    p_rand.setRange(0,1);
    r_rand.setRange(0,1);
    l_rand.setRange(-1,1);
    // rand_index.setRange(0,Ng_-1);

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


bool RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
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
  ROS_INFO("Started computing path with RRT*");

  if (planner_->initialPath(path)) {
    computeInitialPlan(plan, path);
    ROS_INFO("Proceeding to path optimization with WOA");
    if (path.size()>2){
      woaOptimizePath(path, N_, Ng_, b_);
      computeFinalPlan(plan, path);
      ROS_INFO("WOA Executed successfully. New path is published.");
    }
    else {
      ROS_INFO("Path contains only two points. No WOA optimization.");
    }
    return true;

  } else {
    ROS_WARN("The planner failed to find a path, choose other goal position");
    return false;
  }
}

// ---------------------------------
//     ComputeFinalPlan Function
// ---------------------------------

void  RRTStarPlanner::computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,
                                       const std::list<std::pair<float, float>> &path) {
  // clean plan
  plan.clear();
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
    // ROS_INFO("WOA path: Publishing point: (%.4f,%.4f)", point.first, point.second);
  }
  // Publish the path
  nav_msgs::Path path_msg;
  path_msg.header.stamp = plan_time;
  path_msg.header.frame_id = global_frame_;
  path_msg.poses = plan;
  path_pub_.publish(path_msg);
  // ROS_INFO("Published WOA path with %ld points.", plan.size());
}




void  RRTStarPlanner::computeInitialPlan(std::vector<geometry_msgs::PoseStamped>& plan,
                                       const std::list<std::pair<float, float>> &path) {
  // clean plan
  plan.clear();
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
    // ROS_INFO("Initial path: Publishing point: (%.4f,%.4f)", point.first, point.second);
  }
  // Publish the path
  nav_msgs::Path path_msg;
  path_msg.header.stamp = plan_time;
  path_msg.header.frame_id = global_frame_;
  path_msg.poses = plan;
  initial_path_pub_.publish(path_msg);
  ROS_INFO("Published Initial path with %ld points.", plan.size());
}




void RRTStarPlanner::woaOptimizePath(std::list<std::pair<float, float>> &path, int N, int Ng, float spiral_shape) {
  // Initialization
  //---------------
  auto start_time = std::chrono::high_resolution_clock::now();
  // vector containing pointer to all the agents
  // using pointer to avoid moving the object PAthAgent
  // which will make errors since RandomDoubleGenerator is non-movable
  std::vector<std::unique_ptr<PathAgent>> agents;  
  // initial path
  std::list<std::pair<float, float>> initial_path;
  initial_path=path;
  // initialize each agent
  // ROS_INFO("Creating Agent objects");

for (int i = 0; i < Ng; ++i) {
    agents.emplace_back(std::make_unique<PathAgent>(initial_path, sampling_radius_, i, costmap_, spiral_shape));
    // ROS_INFO("Created agent %d", i);
    // constructor: object_name(path, sampling_radius, id, costmap_ptr, b)
    // initialize random path
  }
  ROS_INFO("Created %ld agents", agents.size());
  ROS_INFO("Initialized WOA successfully");
  // Random variables
  float p, r, l, a;
  float A, C;
  int rand; // random agent index
  float best_cost;
  // initialize best cost
  best_cost = agents[0]->fitness();
  // ROS_INFO("Initial best cost %.6f", best_cost);
  // agent size data member
  agent_size_= agents[0]->vec_size;
  arma::vec Xbest; // best agent  
  Xbest.set_size(agent_size_);
  float fitness;
  //---------------
  // Main Loop
  for (int t=0; t<N; t++){
    // ROS_INFO("Iteration number %d", t);
    //---------------
    // Update Xbest
    for (size_t i = 0; i < agents.size(); ++i) {
      // Access the i-th object
      fitness=agents[i]->fitness(); // cost(Xi)
      // ROS_INFO("Cost of %ld-th agent: %lf", i, fitness);
      if (fitness < best_cost){
        // update Xbest if there is a better solution
        Xbest=agents[i]->X;
        best_cost=fitness;
        // ROS_INFO("Current Best cost (iteration %d): %.6f", t, best_cost);
        for (int j=0; j<agent_size_; j+=2){
          // ROS_INFO("Best Agent points (iteration %d): (%.4f,%.4f)", t, Xbest(j), Xbest(j+1));
        }
      }
    }
    // update a
    a=2-2*t/N;

    //---------------
    // Iterate over each agent
    for (int i = 0; i < Ng; ++i) {
      auto& Xi=*agents[i];
      // update the random variables
      p=p_rand.generate();
      r=r_rand.generate();
      // update A and C
      C=2*r;
      A=C*a-a;
      Xi.A=A;
      Xi.C=C;
      if (p<0.5){
        // Circular Search
        if (std::abs(A)>=1){
          // Exploration
          rand=rand_index.generateInt(); // random index
          Xi.circularUpdate(agents[rand]->X); // update Xi using Xrand 
        }
        else{
          // Exploitation
          Xi.circularUpdate(Xbest); // update Xi using Xbest 
        }
      }

      else if (p>=0.5){
        // Spiral Search
        l=l_rand.generate();
        Xi.l=l; // update l
        Xi.spiralUpdate(Xbest);
      }
    }

  }

  // Update Xbest after the end
    for (size_t i = 0; i < agents.size(); ++i) {
      // Access the i-th object
      fitness=agents[i]->fitness(); // cost(Xi)
      // ROS_INFO("Cost of %ld-th agent: %lf", i, fitness);
      if (fitness < best_cost){
        // update Xbest if there is a better solution
        Xbest=agents[i]->X;
        best_cost=fitness;
        // ROS_INFO("Current Best cost: %.4f", best_cost);
      }
    }
  // ROS_INFO("WOA ran for %d iterations", N);
  agentToPath(Xbest, path);
  // ROS_INFO("Start point (%.4f, %.4f)",agents[0]->start_point_.first, agents[0]->start_point_.second);
  // ROS_INFO("Goal point (%.4f, %.4f)",agents[0]->goal_point_.first, agents[0]->goal_point_.second);
  // ROS_INFO("Path Length after WOA: %ld", path.size());
  ROS_INFO("Path Cost after WOA Optimization: %.4f", best_cost);

  // ROS_INFO("Xbest size %d", agent_size_);

  for(int i=0; i<agent_size_; i+=2){
    // ROS_INFO("Best agent %d-th point: (%.4f, %.4f)", i/2+1, Xbest.at(i), Xbest.at(i+1));
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ROS_INFO("Time taken to optimize path with WOA: %f seconds", diff.count());
  
}


/*
Update the actual path (from vector X to Path)
*/ 
void RRTStarPlanner::agentToPath(arma::vec agent, std::list<std::pair<float, float>> &path){
  auto it = path.begin();
  // ROS_INFO("agentToPath: 1st iterator should be start: (%.4f,%.4f)", it->first, it->second);
  for (int i=0; i<agent_size_; i+=2){
      ++it;
      it->first=agent.at(i);
      it->second=agent.at(i+1);
      // ROS_INFO("agentToPath: iterator=(%.4f,%.4f)", agent.at(i), agent.at(i+1));
  } 
  ++it;
  // ROS_INFO("agentToPath: Last iterator should be goal: (%.4f,%.4f)", it->first, it->second);
}



}  // namespace rrt_star_global_planner
