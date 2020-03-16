// Copyright [2020] Yoonyoung (Jamie) Cho

#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "bglpy/sdf.hpp"
#include "bglpy/types.hpp"

namespace cho {
namespace graph {

struct RrtSettings {
  Workspace workspace;
  int max_iter = 512;
  float goal_tol = 1e-3;
  float path_tol = 1e-3;
};

class Rrt {
 public:
  explicit Rrt(const RrtSettings& settings, const SdfFun& dfun);
  ~Rrt();
  std::vector<Node2D> GetTrajectory(const Node2D& source, const Node2D& target);
  void Seed(const std::uint64_t seed);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace graph
}  // namespace cho
