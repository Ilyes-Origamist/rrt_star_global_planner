
#include <pluginlib/class_list_macros.h>

#include "rrt_star_global_planner/rrt_star_planner.hpp"

#include <chrono>
#include <armadillo>
#include <cmath>

// N and Ng are ros parameters  
// b will be a data member for the class PathAgent

namespace woa_based_global_planner {

void  WOAPlanner::woaOptimizePath(const std::list<std::pair<float, float>> &path, int N, int Ng, float spiral_shape) {
  // Initialization
  //---------------
  // vector containing all the agents
  std::vector<PathAgent> agents;
  // initial path
  std::list<std::pair<float, float>> initial_path;
  initial_path=path;
  // initialize each agent
  for (int i = 0; i < Ng; ++i) {
      agents.push_back(PathAgent(path, i, costmap_, spiral_shape)); // create an agent object
      // constructor: object_name(path, id)
      path=randomInitialPath(initial_path); // this function usees "initial_path" 
      // and returns a new path based on it
    }
  // Random variables
  RandomDoubleGenerator r_rand, p_rand, l_rand;
  p_rand.setRange(0,1);
  r_rand.setRange(0,1);
  l_rand.setRange(-1,1);
  RandomIntGenerator rand_index;
  rand_index.setRange(0,Ng-1);
  float p, r, l, a;
  float A, C;
  int rand; // random agent index

  int best=0; // best agent index
  float best_cost=agents[best].cost();
  float fitness;

  //---------------
  // Main Loop
  for (int t=0; t<N; t++){
    
    //---------------
    // Update Xbest
    for (size_t i = 0; i < agents.size(); ++i) {
          // Access the i-th object
        fitness=agents[i].cost(); // cost(Xi)
        if (fitness < best_cost){
          best=i;
          best_cost=agents[best].cost();
        }
    }
    // update a
    a=2-2*t/N;

    //---------------
    // Iterate over each agent
    for (int i = 0; i < Ng; ++i) {
      auto& Xi=agents[i];
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
        if (math::fabs(A)>=1){
          // Exploration
          rand=rand_index.generateInt(); // random index
          Xi.circularUpdate(agents[rand]); // update Xi using Xrand 
        }
        else{
          // Exploitation
          Xi.circularUpdate(agents[best]); // update Xi using Xbest 
        }
      }

      else if (p>=0.5){
        // Spiral Search
        l=l_rand.generate();
        Xi.l=l; // update l
        Xi.spiralUpdate(agents[best]);
      }
    }

  }

}

}  
