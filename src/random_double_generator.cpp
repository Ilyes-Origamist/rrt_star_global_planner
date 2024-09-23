/*
  Copyright 2021 - Rafael Barreto
*/


#include "rrt_star_global_planner/random_double_generator.hpp"

namespace rrt_star_global_planner {

RandomDoubleGenerator::RandomDoubleGenerator() : gen(rd_()) {}

void RandomDoubleGenerator::setRange(double min, double max) {
  min_value_ = min;
  max_value_ = max;
}

double RandomDoubleGenerator::generate() {

  // Note: uniform_real_distribution does [start, stop), but we want to do [start, stop].
  // Therefore passing the next largest value instead.
  return std::uniform_real_distribution<double>{min_value_, std::nextafter(max_value_, DBL_MAX)}(gen);
}

}  // namespace rrt_star_global_planner
