// Copyright [2020] Yoonyoung (Jamie) Cho

#include "bglpy/sdf.hpp"
#include <algorithm>
#include <cmath>

namespace cho {
namespace graph {

SdfFun::SdfFun(){};
SdfFun::SdfFun(const std::function<float(const Node2D&)>& fun) : fun_(fun){};

float SdfFun::operator()(const Node2D& node) const { return fun_(node); };

static const float SquaredDistance(const Node2D& src, const Node2D& dst) {
  const float dx = (src.x - dst.x);
  const float dy = (src.y - dst.y);
  return dx * dx + dy * dy;
}

SdfFun SdfCircle(const Node2D& center, const float radius) {
  return SdfFun([center, radius](const Node2D& target) -> float {
    return std::sqrt(SquaredDistance(center, target)) - radius;
  });
}

SdfFun SdfNegate(const SdfFun& fun) {
  return SdfFun([fun](const Node2D& target) -> float { return -fun(target); });
}

SdfFun SdfSubtract(const SdfFun& a, const SdfFun& b) {
  return SdfFun([a, b](const Node2D& target) -> float {
    return std::max(-a(target), b(target));
  });
}

SdfFun SdfIntersect(const std::vector<SdfFun>& funs) {
  return SdfFun([funs](const Node2D& target) -> float {
    std::vector<float> dists;
    for (const auto& fun : funs) {
      dists.emplace_back(fun(target));
    }
    return *std::max_element(dists.begin(), dists.end());
  });
}

SdfFun SdfUnion(const std::vector<SdfFun>& funs) {
  return SdfFun([funs](const Node2D& target) -> float {
    std::vector<float> dists;
    for (const auto& fun : funs) {
      dists.emplace_back(fun(target));
    }
    return *std::min_element(dists.begin(), dists.end());
  });
}

}  // namespace graph
}  // namespace cho
