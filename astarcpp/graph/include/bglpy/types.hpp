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

}  // namespace graph
}  // namespace cho
