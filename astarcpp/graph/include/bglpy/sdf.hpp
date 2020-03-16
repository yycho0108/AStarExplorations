// Copyright [2020] Yoonyoung (Jamie) Cho

#pragma once

#include <functional>
#include <vector>

#include "bglpy/types.hpp"

namespace cho {
namespace graph {

class SdfFun {
 public:
  SdfFun();
  SdfFun(const std::function<float(const Node2D&)>&);
  float operator()(const Node2D&) const;
  std::function<float(const Node2D&)> fun_;
};

SdfFun SdfCircle(const Node2D& center, const float radius);

SdfFun SdfNegate(const SdfFun& fun);

SdfFun SdfSubtract(const SdfFun& a, const SdfFun& b);

SdfFun SdfIntersect(const std::vector<SdfFun>& funs);

SdfFun SdfUnion(const std::vector<SdfFun>& funs);

}  // namespace graph
}  // namespace cho
