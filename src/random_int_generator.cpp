/*
  Copyright 2021 - Rafael Barreto
*/


#include "rrt_star_global_planner/random_int_generator.hpp"

namespace rrt_star_global_planner {

void RandomIntGenerator::setRange(int min, int max) {
  min_value_ = min;
  max_value_ = max;
}

int RandomIntGenerator::generateInt() {
  std::mt19937 gen(rd_());

  // Note: uniform_real_distribution does [start, stop), but we want to do [start, stop].
  // Therefore passing the next largest value instead.
  return std::uniform_int_distribution<int> {min_value_, std::nextafter(max_value_, DBL_MAX)}(gen);
}

}  // namespace rrt_star_global_planner
