// Copyright [2020] Yoonyoung (Jamie) Cho

#include "bglpy/rrt.hpp"

#include <cmath>
#include <deque>
#include <unordered_map>

#include <iostream>

namespace cho {
namespace graph {

class Rrt::Impl {
  friend class Rrt;

 public:
  explicit Impl(const RrtSettings& settings, const SdfFun& dist_fun);
  ~Impl(){};

 private:
  void Seed(const std::uint64_t seed);
  std::vector<Node2D> GetTrajectory(const Node2D& source,
                                    const Node2D& target) const;

  // Impl. internal
  int GetNearestNodeIndex(const std::vector<Node2D>&, const Node2D&) const;
  bool IsNodeValid(const Node2D&) const;
  float GetExpansionDistance(const Node2D&) const;
  Node2D SampleRandomNode() const;
  bool GetChildNode(const Node2D& parent, const Node2D& target,
                    const float& dist, Node2D* const child) const;

  // Stuff.
  RrtSettings settings_;
  const SdfFun& dist_fun_;
  float min_dist_{0.001f};
};

static const float SquaredDistance(const Node2D& src, const Node2D& dst) {
  const float dx = (src.x - dst.x);
  const float dy = (src.y - dst.y);
  return dx * dx + dy * dy;
}

int Rrt::Impl::GetNearestNodeIndex(const std::vector<Node2D>& sources,
                                   const Node2D& target) const {
  if (sources.empty()) {
    throw std::out_of_range("Current nodes is empty");
  }

  // Naive implementation for now ...
  return std::distance(
      sources.begin(),
      std::min_element(sources.begin(), sources.end(),
                       [&target](const Node2D& lhs, const Node2D& rhs) {
                         return SquaredDistance(lhs, target) <
                                SquaredDistance(rhs, target);
                       }));
}

bool Rrt::Impl::IsNodeValid(const Node2D& node) const {
  return dist_fun_(node) >= 0;
}

float Rrt::Impl::GetExpansionDistance(const Node2D& node) const {
  return std::max(min_dist_, dist_fun_(node));
}

float SampleUniform(const float min_val, const float max_val) {
  return min_val +
         (max_val - min_val) * (static_cast<float>(rand()) / RAND_MAX);
}

Node2D Rrt::Impl::SampleRandomNode() const {
  return Node2D{SampleUniform(settings_.workspace.min_node.x,
                              settings_.workspace.max_node.x),
                SampleUniform(settings_.workspace.min_node.y,
                              settings_.workspace.max_node.y)};
}

bool Rrt::Impl::GetChildNode(const Node2D& parent, const Node2D& target,
                             const float& dist, Node2D* const child) const {
  const float dx = (target.x - parent.x);
  const float dy = (target.y - parent.y);
  const float sqnorm = dx * dx + dy * dy;
  const float norm = std::sqrt(sqnorm);

  if (norm <= std::numeric_limits<float>::epsilon()) {
    return false;
  }

  const float ux = dx / norm;
  const float uy = dy / norm;
  child->x = parent.x + ux * dist;
  child->y = parent.y + uy * dist;
  // Early exit in case of invalid destination.
  if (!IsNodeValid(*child)) {
    return false;
  }
  // Also Check points along the path.
  const float ux1 = ux * settings_.path_tol;
  const float uy1 = uy * settings_.path_tol;
  // NOTE(jamie): std::ceil ?
  const int num_check = std::floor(dist / settings_.path_tol);
  Node2D waypoint{parent.x, parent.y};
  for (int i = 0; i < num_check; ++i) {
    waypoint.x += ux1;
    waypoint.y += uy1;
    // If path intersects with obstacle, return false.
    if (!IsNodeValid(waypoint)) {
      return false;
    }
  }
  return true;
}

void Rrt::Impl::Seed(const std::uint64_t seed) { ::srand(seed); }

std::vector<Node2D> Rrt::Impl::GetTrajectory(const Node2D& source,
                                             const Node2D& target) const {
  std::vector<Node2D> nodes{source};
  std::unordered_map<int, int> parent_map;

  for (int i = 0; i < settings_.max_iter; ++i) {
    const Node2D& random_node = SampleRandomNode();
    const int node_index = GetNearestNodeIndex(nodes, random_node);
    const Node2D& parent_node = nodes[node_index];
    const float expansion_distance = GetExpansionDistance(parent_node);
    Node2D child_node;
    const bool suc =
        GetChildNode(parent_node, random_node, expansion_distance, &child_node);
    if (!suc) {
      continue;
    }

    const int child_index = static_cast<int>(nodes.size());
    parent_map.emplace(child_index, node_index);
    nodes.emplace_back(child_node);

    if (SquaredDistance(child_node, target) <=
        settings_.goal_tol * settings_.goal_tol) {
      std::vector<Node2D> path;

      // Return result.
      int index = child_index;
      while (true) {
        const Node2D& waypoint = nodes[index];
        path.emplace_back(waypoint);

        auto it = parent_map.find(index);
        if (it == parent_map.end()) {
          break;
        }
        index = it->second;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
  }

  return {};
}

Rrt::Rrt(const RrtSettings& settings, const SdfFun& dfun)
    : impl_(std::make_unique<Rrt::Impl>(settings, dfun)) {}

Rrt::~Rrt() = default;

std::vector<Node2D> Rrt::GetTrajectory(const Node2D& source,
                                       const Node2D& target) {
  return impl_->GetTrajectory(source, target);
}

void Rrt::Seed(const std::uint64_t seed) { return impl_->Seed(seed); }

Rrt::Impl::Impl(const RrtSettings& settings, const SdfFun& dfun)
    : settings_(settings), dist_fun_(dfun) {}

}  // namespace graph
}  // namespace cho
