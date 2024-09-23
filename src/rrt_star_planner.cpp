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
    private_nh.param("rewiring_radius", radius_, 0.5);
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
    // p_rand.setRange(0,1);
    r_rand.setRange(0,1);
    l_rand.setRange(-1,1);
    rand_index.setRange(0,Ng_-1);

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
  goal_z = goal.pose.orientation.z;
  goal_w = goal.pose.orientation.w;

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
  std::list<std::pair<float, float>> rand_path;

  // -------------------------------
  // Testing collision detector
  // -------------------------------
  // ROS_INFO("Testing collision detector");
  // std::list<std::pair<float,float>> test_path={{0.0,0.0},{3.0,-1.9}};
  // computeFinalPlan(plan,test_path);
  // CollisionDetector coll_(costmap_);
  // bool collision_test=coll_.isThereObstacleBetween(test_path.front(),test_path.back());
  // if (collision_test==false) ROS_INFO("no collision");
  // else ROS_WARN("There is collision");
  // return false;

  // -------------------------------
  //     RRT* Initial  Path 
  // -------------------------------
  ROS_INFO("Started computing path with RRT*");

  if (planner_->initialPath(path)) {
    computeInitialPlan(plan, path);
    // double time_NN=planner_->time_nearest_neighbour_;
    // ROS_INFO("---> Total time taken by getNearestNodeId: %f seconds", time_NN);
    // -------------------------------
    // Test Agent
    // -------------------------------
    // randomInitialPath test:
    // PathAgent test_agent(path, sampling_radius_, -1, costmap_, b_);
    // Initialization procedure
    // if (path.size()>2){
    //   rand_path=test_agent.randomInitialPath(path);
    //   computeFinalPlan(plan, rand_path);
    //   ROS_INFO("random initial path is published.");
    // }
    // else {
    //   ROS_INFO("Path contains only two points.");
    // }

    // -------------------------------
    //     WOA  Path  Optimization
    // -------------------------------
  
    ROS_INFO("Proceeding to path optimization with WOA for %d iterations", N_);
    if (path.size()>2){
      // multiple tests version
      for (int i=Ng_; i<11*Ng_; i+=10){
        woaOptimizePath(path, N_, i, b_);
        computeFinalPlan(plan, path);
        // ROS_INFO("WOA Executed successfully. New path is published.");
        ROS_INFO("WOA Executed successfully for %d agents.", i);
      }
      // single test version
      // woaOptimizePath(path, N_, Ng_, b_);
      // computeFinalPlan(plan, path);
    }
    else {
      ROS_INFO("Path contains only two points. No WOA optimization.");
    }
    return true;
  }
  
  else {
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
  geometry_msgs::PoseStamped &last_pose = plan.back();
  last_pose.pose.orientation.z = goal_z;
  last_pose.pose.orientation.w = goal_w;

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
  // vector containing pointer to all the agents
  // using pointer to avoid moving the object PAthAgent
  // which will make errors since RandomDoubleGenerator is non-movable
  std::vector<std::unique_ptr<PathAgent>> agents;  
  // initial path
  std::list<std::pair<float, float>> initial_path;
  initial_path=path;
  // initialize each agent
  // ROS_INFO("Creating Agent objects");
  std::vector<geometry_msgs::PoseStamped> plan;
  // ROS_INFO("Initializing whales population");
  auto start_time = std::chrono::high_resolution_clock::now();

for (int i = 0; i < Ng; ++i) {
    agents.emplace_back(std::make_unique<PathAgent>(initial_path, sampling_radius_, i, costmap_, spiral_shape));
    // ROS_INFO("Created agent %d", i);
    // constructor: object_name(path, sampling_radius, id, costmap_ptr, b)
    // initialize random path

    //---------------------------
    // display random initial path for each
    // --------------------------
    // computeFinalPlan(plan, agents[i]->initial_path_);
    // ROS_INFO("Displaying %d-th agent's initial path.", i+1);
    // ros::Duration(0.25).sleep(); 
  }
  ROS_INFO("Created %ld agents", agents.size());
  // ROS_INFO("Initialized WOA successfully");

  // start counting time
  // Random variables
  float p, l, r, a;
  // float p, l, r1, r2, a, a2;
  float A, C;
  int rand; // random agent index
  float best_cost;
  // initialize Xbest with RRT* initial path
  arma::vec Xbest ;
  Xbest = agents[0]->X;
  // initialize best cost
  best_cost = agents[0]->fitness();
  // ROS_INFO("Initial best cost %.6f", best_cost);
  // agent size data member
  agent_size_= agents[0]->vec_size;
  // ROS_INFO("Agent size: %d", agent_size_);
  // arma::vec Xbest; // best agent  
  // Xbest.set_size(agent_size_);


  float fitness;
  // float spiral_collision_rate=0, circular_collision_rate=0;
  // float circular_rate=0;
  // float spiral_rate=0;
  // float circular_exploration_rate=0;
  // float circular_exploitation_rate=0;

  //---------------
  // Main Loop
  for (int t=0; t<N; t++){
    // ROS_INFO("Iteration number %d", t);
    //---------------
    // Update Xbest
    for (size_t i = 0; i < Ng; ++i) {
      // Access the i-th object
      fitness=agents[i]->fitness(); // cost(Xi)
      // ROS_INFO("Cost of %ld-th agent: %lf", i, fitness);
      // !(agents[i]->collides)
      if (fitness < best_cost){
        // update Xbest if there is a better solution that does not collide
        if (!agents[i]->doesPathCollide()){
          Xbest=agents[i]->X;
          best_cost=fitness;
          // ROS_INFO("Current Best cost (iteration %d): %.6f", t, best_cost);
        }
      }
    }
    // display Xbest
    // if (t==0){
    //   for (int j=0; j<agent_size_; j+=2){
    //     ROS_INFO("Initial Best Agent points (iteration %d): (%.4f,%.4f)", t, Xbest(j), Xbest(j+1));
    //   }
    //   ROS_INFO("-----Best Cost: %.4f", best_cost);
    // }
 
    // update a
    // a decreases linearly fron 2 to 0 in Eq. (2.3)
    a=2-2*t/N;

    // a2 linearly dicreases from -1 to -2 to calculate t in Eq. (3.12)
    // a2=-1-t/N;

    //---------------
    // Iterate over each agent
    for (int i = 0; i < Ng; ++i) {
      auto& Xi=*agents[i];
      if (agents[i]==nullptr){
        ROS_WARN("Null pointer on agent %d",i);
      }
      // update the random variables
      p=r_rand.generate();
      r=r_rand.generate();
      // update A and C
      // A=2*a*r1-a;  // Eq. (2.3) in the paper
      // C=2*r2;      // Eq. (2.4) in the paper
      C=2*r;
      A=C*a-a;

      Xi.A=A;
      Xi.C=C;
      
      if (p<0.5){
        // circular_rate+=100.0/(N*Ng);
        // Circular Search
        if (std::abs(A)>=1){
          // Exploration
          // circular_exploration_rate+=100.0/(N*Ng);
          rand=rand_index.generateInt(); // random index
          // ROS_INFO("Rand index: %d",rand);
          // valid pointer check
          // if (agents[rand]==nullptr){
          //   ROS_WARN("Null pointer on the random agent");
          // }
          arma::vec search_ag = agents[rand]->X;
          // for (int k=0; k<agent_size_; k+=2){
          //   ROS_INFO("Xrand %d-th point: (%.4f, %.4f)", k/2+1, search_ag.at(k), search_ag.at(k+1));
          // }
          Xi.circularUpdate(agents[rand]->X); // update Xi using Xrand 
          // if (Xi.collides) circular_collision_rate+=100.0/(N*Ng);
          // if (search_ag.n_elem != agent_size_){
          //   ROS_WARN("Search agent size mismatch");
          // }
        }
        else{
          // Exploitation
          // circular_exploitation_rate+=100.0/(N*Ng);
          Xi.circularUpdate(Xbest); // update Xi using Xbest 
          // if (Xi.collides) circular_collision_rate+=100.0/(N*Ng);
          // for (int k=0; k<agent_size_; k+=2){
          //   ROS_INFO("Xbest %d-th point: (%.4f, %.4f)", k/2+1, Xbest.at(k), Xbest.at(k+1));
          // }
          // if (Xbest.n_elem != agent_size_){
          //   ROS_WARN("Best agent size mismatch");
          // }
        }
      }

      else if (p>=0.5){
        // spiral_rate+=100.0/(N*Ng);
        // Spiral Search
        l=l_rand.generate();
        // l=(a2-1)*r_rand.generate()+1;   //  parameters in Eq. (2.5)
        
        Xi.l=l; // update l
        Xi.spiralUpdate(Xbest);
        // if (Xi.collides) spiral_collision_rate+=100.0/(N*Ng);
        // Xi.circularUpdate(Xbest);
        // for (int k=0; k<agent_size_; k+=2){
        //   ROS_INFO("Xbest %d-th point (spiral): (%.4f, %.4f)", k/2+1, Xbest.at(k), Xbest.at(k+1));
        // }
      }
    }

  }
  // float circular_x_large=0.0, circular_y_large=0.0, spiral_x_large=0.0, spiral_y_large=0.0;
  // Update Xbest after the end
  // float total_time_circular_update_=0;
  // float total_time_spiral_update_=0;
  
  for (size_t i = 0; i < Ng; ++i) {
    // large values rates
    // to check how often do points go far 
    // circular_x_large += static_cast<float>(agents[i]->circular_x_large_) / static_cast<float>(agent_size_*Ng);
    // circular_y_large+=static_cast<float>(agents[i]->circular_y_large_)/static_cast<float>(agent_size_*Ng);
    // spiral_x_large+=static_cast<float>(agents[i]->spiral_x_large_)/static_cast<float>(agent_size_*Ng);
    // spiral_y_large+=static_cast<float>(agents[i]->spiral_y_large_)/static_cast<float>(agent_size_*Ng);

    // Access the i-th object
    fitness=agents[i]->fitness(); // cost(Xi)
    // if (agents[i]->collides) ROS_WARN("Agent %d Actually collides", i);
    // ROS_INFO("Cost of %ld-th agent: %lf", i, fitness);
    if (fitness < best_cost){
      // update Xbest if there is a better solution that does not collide
      if (!agents[i]->doesPathCollide()){
        Xbest=agents[i]->X;
        best_cost=fitness;
        // ROS_INFO("Current Best cost (iteration %d): %.6f", t, best_cost);
      }
    }
    // total_time_circular_update_+=agents[i]->time_circular_update_;
    // total_time_spiral_update_+=agents[i]->time_spiral_update_;
  }

  // ROS_INFO("Circular collision rate: %.4f", circular_collision_rate);
  // ROS_INFO("Spiral collision rate: %.4f", spiral_collision_rate);
  // -----------------------------------------------------------------------
  // Collision test
  // bool collides=false;
  // std::pair<float, float> start_point_= path.front();
  // std::pair<float, float> goal_point_ = path.back();
  // CollisionDetector collision_(costmap_);
  // // if 1st point collides or obstacle between 1st point and start_point_
  // if (collision_.isThisPointCollides(Xbest.at(0), Xbest.at(1)) || collision_.isThereObstacleBetween(start_point_, std::make_pair(Xbest.at(0), Xbest.at(1)))){
  //   collides=true;
  //   ROS_WARN("Start point collides with next point");
  // }
  // // if last point collides or obstacle between last point and goal point 
  // if (collision_.isThisPointCollides(Xbest.at(agent_size_-2), Xbest.at(agent_size_-1)) || collision_.isThereObstacleBetween(goal_point_, std::make_pair(Xbest.at(agent_size_-2), Xbest.at(agent_size_-1)))){
  //   collides=true;
  //   ROS_WARN("Goal point collides with previous point");
  // }

  // // collision check for each new point in Xbest and between its preceeding point 
  // int k=2;

  // while (!collides && k<agent_size_){
  //   if (collision_.isThisPointCollides(Xbest.at(k), Xbest.at(k+1))){
  //     collides=true;
  //   }
  //   if (collision_.isThereObstacleBetween(std::make_pair(Xbest.at(k-2), Xbest.at(k-1)), std::make_pair(Xbest.at(k), Xbest.at(k+1)))){
  //     collides=true;
  //   }
    
  //   // boundary check X
  //   if(Xbest.at(k)>map_width_){
  //     Xbest.at(k)=map_width_;
  //     ROS_WARN("Xbest (x) is too large");
  //   }

  //   // boundary check Y
  //   if(Xbest.at(k+1)>map_height_){
  //     Xbest.at(k+1)=map_height_;
  //     ROS_WARN("Xbest (y) is too large");
  //   }

  //   k+=2;
  // }

  // if(collides) ROS_WARN("There is collision in the path found.");
  // else ROS_INFO("No collision found on the path.");
  // -----------------------------------------------------------------------


  // ROS_INFO("Circular Exploration rate = %.2f", circular_exploration_rate);
  // ROS_INFO("Circular Exploitation rate = %.2f", circular_exploitation_rate);
  // ROS_INFO("Circular rate = %.2f", circular_rate);
  // ROS_INFO("Spiral rate = %.2f", spiral_rate);

  // ROS_INFO("WOA ran for %d iterations", N);
  agentToPath(Xbest, path);
  // ROS_INFO("Start point (%.4f, %.4f)",agents[0]->start_point_.first, agents[0]->start_point_.second);
  // ROS_INFO("Goal point (%.4f, %.4f)",agents[0]->goal_point_.first, agents[0]->goal_point_.second);
  // ROS_INFO("Path Length after WOA: %ld", path.size());
  ROS_INFO("Path Cost after WOA Optimization: %.4f", best_cost);

  // ROS_INFO("Circular X large occurance average: %.2f", circular_x_large);
  // ROS_INFO("Circular Y large occurance average: %.2f", circular_y_large);
  // ROS_INFO("Spiral X large occurance average: %.2f", spiral_x_large);
  // ROS_INFO("Spiral Y large occurance average: %.2f", spiral_y_large);
  // ROS_INFO("Total large values occurance relative to number of iterations: %.4f",(circular_x_large+circular_y_large+spiral_x_large+spiral_y_large)*100/N);
  // // ROS_INFO("Xbest size %d", agent_size_);

  // for(int i=0; i<agent_size_; i+=2){
  //   ROS_INFO("Best agent %d-th point: (%.4f, %.4f)", i/2+1, Xbest.at(i), Xbest.at(i+1));
  // }
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ROS_INFO("---> Time taken to optimize path with WOA: %f seconds", diff.count());
  // ROS_INFO("---> Time taken by CIRCULAR UPDATE: %.4f seconds", total_time_circular_update_);
  // ROS_INFO("---> Time taken by SPIRAL UPDATE: %.4f seconds", total_time_spiral_update_);
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
