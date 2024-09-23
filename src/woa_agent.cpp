

#include "rrt_star_global_planner/woa_agent.hpp"
#include <costmap_2d/costmap_2d.h>
#include <iostream>

namespace rrt_star_global_planner {

// Initializer list

PathAgent::PathAgent(std::list<std::pair<float, float>> &path,
                     const float sampling_radius,
                     uint16_t id,
                     costmap_2d::Costmap2D* costmap,
                     float spiral_shape): id_(id),
                                          path_(path),
                                          sampling_radius_(sampling_radius),
                                          collision_(costmap),
                                          b(spiral_shape),
                                          costmap_(costmap){

    map_width_=costmap_->getSizeInMetersX();
    map_height_=costmap_->getSizeInMetersY();
        // ROS_INFO("PathAgent constructor started");
        //   // Initialization code
        // ROS_INFO("PathAgent initialized parameters");
        //   // Additional setup
        // ROS_INFO("PathAgent setup complete");

    // initialize path
    if (id_==0) initial_path_=path_;
    else{
      path_ = randomInitialPath(path);
      initial_path_=path_;
    }

    start_point_= path_.front();
    goal_point_ = path_.back();
    // ROS_INFO("Start point is (%.4f, %.4f)", start_point_.first, start_point_.second);
    // ROS_INFO("Goal point is (%.4f, %.4f)", goal_point_.first, goal_point_.second);

    // X exclude start and goal points. they are fixed points
    vec_size = static_cast<uint16_t>(2*path_.size()-4);
    // Initialize X
    X.set_size(vec_size);
    auto it = path_.begin();
    ++it;  // Use path_ here
    for (int i = 0; i < vec_size; i += 2) {
        X.at(i) = static_cast<float>(it->first);
        X.at(i+1) = static_cast<float>(it->second);
        // ROS_INFO("Initial Agent Path: point %d-th: (%.4f, %.4f)", i/2+1, X.at(i), X.at(i+1));
        ++it;  // Move to the next element in the list
    }

    // D.set_size(vec_size);
    // D2.set_size(vec_size);
    arma::vec D = arma::zeros<arma::vec>(vec_size); 
    arma::vec D2 = arma::zeros<arma::vec>(vec_size); 
    // ROS_INFO("Agent Size: %d", vec_size);
    // ROS_INFO("Done initializing");
  // circular_x_large_=0;
  // circular_y_large_=0;
  // spiral_x_large_=0;
  // spiral_y_large_=0;
}


// PathAgent::PathAgent(PathAgent&& other) noexcept
//     : path_(std::move(other.path_)),
//       cd_(std::move(other.cd_)),
//       D(std::move(other.D)),
//       D2(std::move(other.D2)),
//       random_device_(std::move(other.random_device_)),
//       id_(other.id_),
//       X(std::move(other.X)),
//       vec_size(other.vec_size),
//       b(other.b) {
//   // Any additional initialization code here
// }

// PathAgent& PathAgent::operator=(PathAgent&& other) noexcept {
//   if (this != &other) {
//     path_ = std::move(other.path_);
//     cd_ = std::move(other.cd_);
//     D = std::move(other.D);
//     D2 = std::move(other.D2);
//     random_device_ = std::move(other.random_device_);
//     id_ = other.id_;
//     X = std::move(other.X);
//     vec_size = other.vec_size;
//     b = other.b;
//   }
//   return *this;
// }


// // Move constructor
// PathAgent::PathAgent(PathAgent&& other) noexcept
//     : path_(other.path_),
//       cd_(std::move(other.cd_)),
//       D(std::move(other.D)),
//       D2(std::move(other.D2)),
//       random_device_(), // Reinitialize this member
//       id_(other.id_),
//       X(std::move(other.X)),
//       vec_size(other.vec_size),
//       b(other.b) {
//   other.path_ = nullptr;  // Nullify other's pointer
// }

// // Move assignment operator
// PathAgent& PathAgent::operator=(PathAgent&& other) noexcept {
//   if (this != &other) {
//     path_ = other.path_;
//     cd_ = std::move(other.cd_);
//     D = std::move(other.D);
//     D2 = std::move(other.D2);
//     random_device_ = RandomDoubleGenerator();  // Reinitialize this member
//     id_ = other.id_;
//     X = std::move(other.X);
//     vec_size = other.vec_size;
//     b = other.b;
//     other.path_ = nullptr;  // Nullify other's pointer
//   }
//   return *this;
// }

/*
circular update
*/
void PathAgent::circularUpdate(arma::vec search_agent) {
  auto start_time = std::chrono::high_resolution_clock::now();
  arma::vec D=arma::abs(C*search_agent-X);
  // ROS_INFO("A=%.4f", A);
  // ROS_INFO("C=%.4f", C);

  // for(int k=0; k<vec_size; k+=2){
  //   ROS_INFO("D %d-th element is: (%.4f, %.4f)", k/2+1, D.at(k), D.at(k+1));
  // }
  // for(int k=0; k<vec_size; k+=2){
  //   ROS_INFO("Search agent %d-th element is: (%.4f, %.4f)", k/2+1, search_agent.at(k), search_agent.at(k+1));
  // }
  arma::vec Xnew;
  Xnew=search_agent-A*D;
  // ROS_INFO("Xnew before collision check");
  // for(int k=0; k<vec_size; k+=2){
  //   ROS_INFO("Xnew %d-th element is: (%.4f, %.4f)", k/2+1, Xnew.at(k), Xnew.at(k+1));
  // }

  if (Xnew.has_nan()) {
      ROS_ERROR("Xnew contains NaN values!");
  }
  if (Xnew.has_inf()) {
      ROS_ERROR("Xnew contains Inf values!");
  }

  // boundary check 
  for (int k=0; k<vec_size; k+=2){
    // boundary check X
    if(Xnew.at(k)>map_width_){
      Xnew.at(k)=map_width_;
      // circular_x_large_+=1;
      // ROS_WARN("Xnew (x) is too large");
    }
    // boundary check Y
    if(Xnew.at(k+1)>map_height_){
      Xnew.at(k+1)=map_height_;
      // circular_y_large_+=1;
      // ROS_WARN("Xnew (y) is too large");
    }
  }

  // // update X despite colliding
  X=Xnew;

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  time_circular_update_ += diff.count();

  // Collision test
  // if 1st point collides or obstacle between 1st point and start_point_
  
  // if new path does not pass by any obstacle
  // if (!collides){
  //   // update X
  //   X=Xnew;
  //   // display X
  //   // for (int j=0; j<vec_size; j+=2){
  //   //   ROS_INFO("Agent circular update: %d-th point: (%.4f, %.4f)", k/2+1, X.at(j), X.at(j+1));
  //   // }
  // }
  }


/*
spiral update
*/
void PathAgent::spiralUpdate(arma::vec search_agent) {
  auto start_time = std::chrono::high_resolution_clock::now();
  arma::vec D2=arma::abs(search_agent-X);
  arma::vec Xnew;
  Xnew=search_agent+(std::exp(b*l)*std::cos(2*M_PI*l))*D2;

  // check for invalid values
  if (Xnew.has_nan()) {
      ROS_ERROR("Xnew contains NaN values!");
  }
  if (Xnew.has_inf()) {
      ROS_ERROR("Xnew contains Inf values!");
  }

  // boundary check 
  for (int k=0; k<vec_size; k+=2){
    // boundary check X
    if(Xnew.at(k)>map_width_){
      Xnew.at(k)=map_width_;
      // spiral_x_large_+=1;
      // ROS_WARN("Xnew (x) is too large");      
    }
    // boundary check Y
    if(Xnew.at(k+1)>map_height_){
      Xnew.at(k+1)=map_height_;
      // spiral_y_large_+=1;
      // ROS_WARN("Xnew (y) is too large");
    }
  }

  // // update X despite colliding
  X=Xnew;

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  time_spiral_update_ += diff.count();


  // Collision test
  
  // if new path does not pass by any obstacle
  // if (!collides){
  //   // update X
  //   X=Xnew;
  //   // display X
  //   // for (int j=0; j<vec_size; j+=2){
  //   //   ROS_INFO("Agent circular update: %d-th point: (%.4f, %.4f)", k/2+1, X.at(j), X.at(j+1));
  //   // }
  // }
}

float PathAgent::fitness() {
    float cost=0;
    // euclideanDistance2D(x1,y1,x2,y2)
    cost+=euclideanDistance2D(start_point_.first, start_point_.second, X.at(0), X.at(1));
    // ROS_INFO("Adding distance between the two points: start (%.4f, %.4f) and (%.4f, %.4f)", start_point_.first, start_point_.second, X.at(0), X.at(1));
    // ROS_INFO("Cost after adding the start point: %.4f", cost);
    if (vec_size>2){
      for(int i=0; i<vec_size-2; i+=2){
        cost+=euclideanDistance2D(X.at(i), X.at(i+1), X.at(i+2), X.at(i+3));
          // ROS_INFO("Adding distance between the two points: (%.4f, %.4f) and (%.4f, %.4f)", X.at(i), X.at(i+1), X.at(i+2), X.at(i+3));
          // ROS_INFO("Cost after adding the %d-th point: %.4f",i/2+1, cost);
      }
    }
    cost+=euclideanDistance2D(goal_point_.first, goal_point_.second, X.at(vec_size-2), X.at(vec_size-1));
    // ROS_INFO("Adding distance between the two points: goal (%.4f, %.4f) and (%.4f, %.4f)", goal_point_.first, goal_point_.second, X.at(vec_size-2), X.at(vec_size-1));
    // ROS_INFO("Cost after adding the goal point: %.4f", cost);
    if (cost<0) {
      ROS_WARN("Cost is negative");
    }
    return cost;
}

// i= 0  1  2  3  4  5  6  7
//    x1 y1 x2 y2 x3 y3 x4 y4
// iter 1: dist(P1, P2) 
// iter 2: dist(P2, P3)
// iter n-4: dist(Pn-4, Pn-2)


/*
Index of the agent (id)
*/ 
uint16_t PathAgent::getID(){
  return id_;
}


/*
doesPathCollides
*/
bool PathAgent::doesPathCollide(){
  // if 1st point collides or obstacle between 1st point and start_point_
  if (collision_.isThisPointCollides(X.at(0), X.at(1)) || collision_.isThereObstacleBetween(start_point_, std::make_pair(X.at(0), X.at(1)))){
    return true;
    // ROS_WARN("Start point collides with next point (spiral)");
  }
  // if last point collides or obstacle between last point and goal point 
  if (collision_.isThisPointCollides(X.at(vec_size-2), X.at(vec_size-1)) || collision_.isThereObstacleBetween(goal_point_, std::make_pair(X.at(vec_size-2), X.at(vec_size-1)))){
    return true;
    // ROS_WARN("Goal point collides with previous point (spiral)");
  }

  // collision check for each new point in Xnew and between its preceeding point 
  int k=2;
  
  while (k<vec_size){
    if (collision_.isThisPointCollides(X.at(k), X.at(k+1))){
      return true;
      // ROS_WARN("There is collision in Xnew (spiral)");
    }
    if (collision_.isThereObstacleBetween(std::make_pair(X.at(k-2), X.at(k-1)), std::make_pair(X.at(k), X.at(k+1)))){
      return true;
      // ROS_WARN("There is collision in Xnew (spiral)");
    }
    k+=2;
  }
  return false;
}


/*
radnom initial path
*/
std::list<std::pair<float, float>> PathAgent::randomInitialPath(std::list<std::pair<float,float>> &path) {
  std::list<std::pair<float, float>> rand_path;
  std::pair<float, float> p_rand;
  std::pair<float, float> p_new;
  // std::pair<float, float> current_point;
  // Node node_nearest;
  bool found_next=false;
  int num_travels_=0;
  auto it = path.begin();  // Iterator to traverse the list
  rand_path.push_front(*it);
  // ROS_INFO("First iterator: (%.4f, %.4f)", it->first, it->second);
  ++it;
  int i=2;
  // starting from the second node

  // main loop
  while (it != path.end()) {
    // ROS_INFO("Current iterator from initial path: (%.4f, %.4f)", it->first, it->second);
    found_next = false; // Reset found_next before the inner loop
    while (!found_next) {
      // current node = path[i] when traveling
      // biased sampling with current node as center of the circle   
      p_rand= biasedSampling(*it);
      auto prev_it = std::prev(it);   
      // std::cout << "Type of *prev_it: " << typeid(*prev_it).name() << std::endl;
      // std::cout << "Type of p_rand: " << typeid(p_rand).name() << std::endl;

      // Ensure *prev_it and p_rand are of type std::pair<double, double>
      auto prev_point = std::make_pair(static_cast<double>(prev_it->first), static_cast<double>(prev_it->second));
      auto rand_point = std::make_pair(static_cast<double>(p_rand.first), static_cast<double>(p_rand.second));

      if (!collision_.isThereObstacleBetween(prev_point, rand_point)) {
        found_next = true;
        rand_path.push_back(rand_point);
        // ROS_INFO("Adding point from randomInitialPath: (%.4f, %.4f)", rand_point.first, rand_point.second);
      }
      // else loop until found_next=true
    }
    // ROS_INFO("Just processed the %d-th element", i);
    i++;
    // moving to next node
    ++it;
    }
    // ROS_INFO("Last iterator (after loop): (%.4f, %.4f)", it->first, it->second);
    // ROS_INFO("Now number of last element: %d", i);
    // rand_path.push_back(*it); // last point
    // rand_path.push_back(std::make_pair(it->first, it->second)); // last point
    // ROS_INFO("Path length %ld", rand_path.size());
    return rand_path;
  }


/*
biasedSampling
*/
std::pair<float, float> PathAgent::biasedSampling(std::pair<double, double> center) {
  std::pair<float, float> new_rand_point;  
  double min_x = center.first - sampling_radius_;
  double max_x = center.first + sampling_radius_;
  double min_y = center.second - sampling_radius_;
  double max_y = center.second + sampling_radius_;
  random_device_.setRange(min_x,max_x);
  new_rand_point.first = random_device_.generate();
  random_device_.setRange(min_y,max_y);
  new_rand_point.second = random_device_.generate();
  
  return new_rand_point;
}

// void PathAgent::updatePath() {
//     // Ensure that X has the correct size
//     if (X.n_elem != vec_size) {
//         throw std::runtime_error("Size of X does not match vec_size");
//     }

//     // Create an iterator to the beginning of the path_ list
//     auto it = path_.begin();

//     // Iterate over the elements in X and update the path_
//     for (int i = 0; i < vec_size; i += 2) {
//         if (it == path_.end()) {
//             throw std::runtime_error("Path length exceeds the size of path_");
//         }
        
//         it->first = X(i);     // Update the first element of the pair
//         it->second = X(i + 1); // Update the second element of the pair
//         ++it; // Move to the next element in the path_
//     }

//     // Check if the iterator has reached the end of path_
//     if (it != path_.end()) {
//         throw std::runtime_error("Path length is less than the size of X");
//     }
// }

}