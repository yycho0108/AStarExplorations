// Copyright [2020] Yoonyoung (Jamie) Cho

#pragma once

namespace cho {
namespace graph {

class Node2D {
 public:
  float x, y;
  Node2D();
  Node2D(float x, float y);
  Node2D(const Node2D& o);
};

class Edge2D {
 public:
  int source;
  int target;
};

}  // namespace graph
}  // namespace cho
