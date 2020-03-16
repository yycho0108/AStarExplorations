// Copyright [2020] Yoonyoung (Jamie) Cho

#include <fmt/format.h>
#include <Eigen/Core>

#include <boost/functional/hash.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>

#include <unordered_map>

#include "bglpy/graph.hpp"
#include "bglpy/rrt.hpp"

static const float SquaredDistance(const cho::graph::Node2D& src,
                                   const cho::graph::Node2D& dst) {
  const float dx = (src.x - dst.x);
  const float dy = (src.y - dst.y);
  return dx * dx + dy * dy;
}

float WorkspaceDistance(const cho::graph::Node2D& node) {
  using Point = cho::graph::Node2D;
  static constexpr const float kHipJointOffset = 0.0110;
  static constexpr const float kKneeLinkLength = 0.0285;
  static constexpr const float kHipLinkLength = 0.0175;

  static constexpr const float kSmallRadius = kKneeLinkLength - kHipLinkLength;
  static constexpr const float kLargeRadius = kKneeLinkLength + kHipLinkLength;

  const Point center_a{kHipJointOffset, 0.0f};
  const Point center_b{-kHipJointOffset, 0.0f};
  const float radius_a = std::sqrt(SquaredDistance(node, center_a));
  const float radius_b = std::sqrt(SquaredDistance(node, center_b));

  const std::array<float, 4> distances{
      {radius_a - kSmallRadius, radius_b - kSmallRadius,
       kLargeRadius - radius_a, kLargeRadius - radius_b}};
  const float dist = *std::min_element(distances.begin(), distances.end());
  return dist;
}

int main(const int argc, const char* const argv[]) {
  const cho::graph::Workspace ws{{-0.035, -0.044}, {0.035, 0.044}};
  const cho::graph::RrtSettings settings{ws, 512, 1e-3, 1e-3};
  cho::graph::Rrt rrt{settings, cho::graph::SdfFun(WorkspaceDistance)};
  const cho::graph::Node2D& source{0.0032096, 0.03201624};
  const cho::graph::Node2D& target{0.01301248, -0.01502493};
  const std::vector<cho::graph::Node2D>& path =
      rrt.GetTrajectory(source, target);
  fmt::print("Source : {} {}\n", source.x, source.y);
  fmt::print("Target : {} {}\n", target.x, target.y);
  for (const auto& point : path) {
    fmt::print("{} {}\n", point.x, point.y);
  }

  // Eigen::Matrix<std::uint8_t, Eigen::Dynamic, Eigen::Dynamic> grid =
  // decltype(grid)::Zero(3, 5); const cho::VecVector2i& path =
  // cho::graph::FindPath(grid, { 0, 0 }, { 2, 4 }); for (const auto& v : path)
  // {
  //    fmt::print("P {} {}\n", v.x(), v.y());
  //}
  return 0;
}
