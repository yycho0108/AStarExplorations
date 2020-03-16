// Copyright [2020] Yoonyoung (Jamie) Cho

#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "sdf.hpp"
#include "types.hpp"

namespace cho {
namespace graph {

/**
 * Maximal bounding rectangular workspace.
 * If this is meant to be general, could
 */
struct Workspace {
  Node2D min_node;
  Node2D max_node;
  Workspace() {}
  Workspace(const Node2D& min, const Node2D& max)
      : min_node(min), max_node(max) {}
  Workspace(const Workspace& ws)
      : min_node(ws.min_node), max_node(ws.max_node) {}
};

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
