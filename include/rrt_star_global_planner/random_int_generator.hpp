/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_RANDOM_INT_GENERATOR_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_RANDOM_INT_GENERATOR_HPP_

#include <random>
#include <cfloat>  // DBL_MAX

namespace rrt_star_global_planner {

// TODO(Rafael) allow different ranges of x and y for non square maps

class RandomIntGenerator {
 private:
  std::random_device rd_;
  int min_value_{0};
  int max_value_{1};

 public:
  RandomIntGenerator() = default;

  void setRange(int min, int max);
  int generateInt();
};
}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RANDOM_DOUBLE_GENERATOR_HPP_  // NOLINT
