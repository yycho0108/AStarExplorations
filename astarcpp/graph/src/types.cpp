// Copyright [2020] Yoonyoung (Jamie) Cho

#include "bglpy/types.hpp"

namespace cho {
namespace graph {

Node2D::Node2D() {}
Node2D::Node2D(float x, float y) : x(x), y(y) {}
Node2D::Node2D(const Node2D& o) : x(o.x), y(o.y) {}

}  // namespace graph
}  // namespace cho
