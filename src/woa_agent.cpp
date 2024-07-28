

#include "rrt_star_global_planner/woa_agent.hpp"


namespace rrt_star_global_planner {

// Initializer list

PathAgent::PathAgent(std::list<std::pair<float, float>> &path,
                     uint16_t id,
                     costmap_2d::Costmap2D* costmap,
                     float spiral_shape): id_(id),
                                          path_(&path),
                                          cd_(costmap),
                                          b(spiral_shape){
    // initialize path
    path_ = randomInitialPath(&path);
    vec_size = static_cast<uint16_t>(2*path_.size());
    // Initialize X
    X.set_size(vec_size);
    auto it = path_.begin();  // Use path_ here
    for (int i = 0; i < vec_size; i += 2) {
        X(i) = static_cast<float>(it->first);
        X(i+1) = static_cast<float>(it->second);
        ++it;  // Move to the next element in the list
    }
    D.set_size(vec_size);
    D2.set_size(vec_size);
}


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



void PathAgent::circularUpdate(PathAgent &search_agent) {
    D=arma::abs(C*search_agent.X-X);
    X=search_agent.X-A*D;
}

void PathAgent::spiralUpdate(PathAgent &search_agent) {
    D2=arma::abs(search_agent.X-X);
    X=search_agent.X+std::exp(b*l)*std::cos(2*M_PI*l)*D2;
}

float PathAgent::fitness() {
    float cost=0;
    for(int i=0; i<vec_size-2; i+=2){
        cost+=euclideanDistance2D(X(i), X(i+1), X(i+2), X(i+3));
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
Get the actual path
*/ 
std::list<std::pair<float, float>> PathAgent::getPath(){
    updatePath();
    return path_;
}

/*
Update the actual path (from vector X to Path)
*/ 
void PathAgent::updatePath(){
    auto it = path_.begin(); 
    for (int i=0; i<vec_size; i+=2){
        it->first=X(i);
        it->second=X(i+1);
        ++it;
    }
}


/*
radnom initial path
*/
std::list<std::pair<float, float>> PathAgent::randomInitialPath(std::list<std::pair<float, float>> &path) {
  std::list<std::pair<float, float>> rand_path;
  std::pair<float, float> p_rand;
  std::pair<float, float> p_new;
  // std::pair<float, float> current_point;
  Node node_nearest;
  bool found_next=false;
  int num_travels_=0;
  auto it = path.begin();  // Iterator to traverse the list
  rand_path.emplace_back((it->first,it->second));
  ++it;
  // starting from the second node

  // main loop
  while (it != path.end()) {
    found_next = false; // Reset found_next before the inner loop
    while (!found_next) {
      // current node = path[i] when traveling
      // biased sampling with current node as center of the circle   
      p_rand= biasedSampling(*it);
      auto prev_it = std::prev(it);     
      if (!cd_.isThereObstacleBetween(*prev_it, p_rand)) {
        found_next = true;
        rand_path.emplace_back((p_rand.first, p_rand.second));      
      }
      // else loop until found_next=true
    }
      // moving to next node
      ++it;
    }
    rand_path.emplace_back((it->first,it->second)); // last point
    ROS_INFO("Path length %ld", rand_path.size());
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