// Copyright [2020] Yoonyoung (Jamie) Cho

#include "bglpy/graph.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>

#include <iostream>

#include <fmt/format.h>

namespace cho {
namespace graph {
// Define all the type aliases ...
struct Node2D {
  float x;
  float y;
};

struct Edge2D {
  /* float weight{std::numeric_limits<float>::max()}; */
  float weight;
};

struct found_goal {};

template <typename VertexProperty = boost::no_property,
          typename EdgeProperty = boost::property<boost::edge_weight_t, float>>
using GraphBase =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                          VertexProperty, EdgeProperty>;
using Graph2D = GraphBase<Node2D, Edge2D>;

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor {
 public:
  astar_goal_visitor(const Vertex& goal) : m_goal(goal) {}
  template <class Graph>
  void examine_vertex(const Vertex& u, Graph& g) {
    // fmt::print("v{},{}\n", u[0], u[1]);
    if (u == m_goal) {
      throw found_goal();
    }
  }

 private:
  Vertex m_goal;
};

template <class Graph, class Vertex, class LocMap, class CostType>
class distance_heuristic : public boost::astar_heuristic<Graph, CostType> {
 public:
  distance_heuristic(const LocMap& loc_map, const Vertex& goal)
      : loc_map_(loc_map), goal_(goal) {}
  CostType operator()(const Vertex& u) {
    const auto& v0 = loc_map_(u);
    const auto& v1 = loc_map_(goal_);
    // fmt::print("H{} {} - {} {}\n", v0.first, v0.second, v1.first, v1.second);
    const float& dx = v0.first - v1.first;
    const float& dy = v0.second - v1.second;
    return std::sqrt(dx * dx + dy * dy);
  }

 private:
  LocMap loc_map_;
  Vertex goal_;
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    // Note that it is oblivious to the storage order of Eigen matrix (column-
    // or row-major). It will give you the same hash value for two different
    // matrices if they are the transpose of each other in different storage
    // order.
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

auto BuildGridGraph(const MatrixXu& grid) {
  boost::array<int, 2> dims({int(grid.rows()), int(grid.cols())});
  auto g_ptr = std::make_shared<boost::grid_graph<2, int, int>>(dims);
  auto& g = *g_ptr;

  using Graph = std::remove_reference<decltype(g)>::type;
  using Vertex = Graph::vertex_descriptor;
  using VertexSet = boost::unordered_set<Vertex>;
  using FGraph = boost::vertex_subset_complement_filter<Graph, VertexSet>::type;

  std::shared_ptr<VertexSet> barriers = std::make_shared<VertexSet>();

  std::unordered_map<Eigen::Vector2i, Vertex, matrix_hash<Eigen::Vector2i>>
      ivmap;
  for (int i = 0; i < grid.size(); ++i) {
    const auto& v = vertex(i, g);
    // iv0.y() * grid.rows() + iv0.x()

    const int x = i % grid.rows();
    const int y = i / grid.rows();
    if (grid(x, y)) {
      barriers->insert(v);
    }

    ivmap.emplace(Eigen::Vector2i{x, y}, v);
  }
  auto graph = std::make_shared<FGraph>(
      boost::make_vertex_subset_complement_filter(g, *barriers));

  // NOTE(jamie): barriers must be returned here,
  // since make_vertex_subset_complement_filter does not track barriers.
  return std::make_tuple(g_ptr, graph, ivmap, barriers);
}

auto BuildGraph(
    const MatrixXu& grid, cho::graph::Graph2D* const g_ptr,
    std::unordered_map<
        Eigen::Vector2i,
        std::remove_reference<decltype(*g_ptr)>::type::vertex_descriptor,
        matrix_hash<Eigen::Vector2i>>* const ivmap) {
  // Type alias
  using Graph = std::remove_reference<decltype(*g_ptr)>::type;
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
  using Edge = typename boost::graph_traits<Graph>::edge_descriptor;

  auto& g = *g_ptr;
  g.m_vertices.reserve(grid.size());

  // Separately track (index -> vertex) map.
  std::unordered_map<Eigen::Vector2i, Vertex, matrix_hash<Eigen::Vector2i>>
      vmap;

  // Add Nodes first.
  for (int i = 0; i < grid.rows(); ++i) {
    for (int j = 0; j < grid.cols(); ++j) {
      if (!grid(i, j)) {
        continue;
      }
      const Vertex& src = boost::add_vertex(g);
      g[src].x = i;
      g[src].y = j;
      vmap.emplace(Eigen::Vector2i{i, j}, src);
    }
  }

  if (ivmap) {
    *ivmap = vmap;
  }

  // Add Edges.
  // g.m_edges.reserve(8 * grid.rows() * grid.cols());
  for (int i = 0; i < grid.rows(); ++i) {
    for (int j = 0; j < grid.cols(); ++j) {
      if (!grid(i, j)) {
        continue;
      }
      for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
          const int i2 = i + di;
          const int j2 = j + dj;
          // Skip out-of-bounds
          if (i2 < 0 || i2 >= grid.rows() || j2 < 0 || j2 >= grid.cols()) {
            continue;
          }

          // Skip self.
          if (i == i2 && j == j2) {
            continue;
          }

          // Skip empty cell.
          if (!grid(i2, j2)) {
            continue;
          }

          // Add that edge.
          /* fmt::print("E {},{} - {},{}\n", i, j, i2, j2); */

          const Edge& e =
              boost::add_edge(vmap.at({i, j}), vmap.at({i2, j2}), g).first;
          g[e].weight = (di * di + dj * dj);
        }
      }
    }
  }

  return g;
}
VecVector2i FindPath(const MatrixXu& grid, const Eigen::Vector2i& iv0,
                     const Eigen::Vector2i& iv1) {
  // fmt::print(" Input [I-J] Order ");
  // fmt::print(" {} {} -> {} {}", iv0.x(), iv0.y(), iv1.x(), iv1.y());

  // using Graph = cho::graph::Graph2D;
  // using Vertex = Graph::vertex_descriptor;

  // Graph g
  // std::unordered_map<Eigen::Vector2i, Vertex, matrix_hash<Eigen::Vector2i>>
  // ivmap; BuildGraph(grid, &g, &ivmap);

  auto e = BuildGridGraph(grid);
  auto& g0 = *std::get<0>(e);
  auto& g1 = *std::get<1>(e);
  // auto& ivmap = std::get<2>(e);
  auto& barrier = std::get<3>(e);

  using Graph = std::remove_reference<decltype(g1)>::type;
  // using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
  using Vertex = std::remove_reference<decltype(g0)>::type::vertex_descriptor;
  boost::static_property_map<float> wmap(1);

  // for (int i = 0; i < num_vertices(g0); ++i) {
  //    const auto& v = vertex(i, g0);
  //    if (i == num_vertices(g0) - 1) {
  //        // fmt::print("last : {} {}\n", v[0], v[1]);
  //    }
  //}

  // THIS IS ABSOLUTELY THE MOST CONFUSING PART.
  // vertex id is incremented 0-dim first for whatever reason...
  // vid = iv0.x() * grid.c
  const Vertex& v0 = vertex(iv0.y() * grid.rows() + iv0.x(), g0);
  const Vertex& v1 = vertex(iv1.y() * grid.rows() + iv1.x(), g0);

  // std::vector<Vertex> prv_map(boost::num_vertices(g));
  // std::vector<float> d(boost::num_vertices(g));
  boost::unordered_map<Vertex, Vertex, boost::hash<Vertex>> prv_map;
  boost::associative_property_map<decltype(prv_map)> prv_pmap(prv_map);

  boost::unordered_map<Vertex, float, boost::hash<Vertex>> dist_map;
  boost::associative_property_map<
      boost::unordered_map<Vertex, float, boost::hash<Vertex>>>
      dist_pmap(dist_map);

  boost::unordered_map<Vertex, std::pair<float, float>, boost::hash<Vertex>>
      loc_map;

  const auto& GetXy = [](const Vertex& v) -> auto {
    return std::make_pair(v[0], v[1]);
    // const int idx = boost::get(boost::vertex_index, g1, v);
    // int x = idx / grid.cols();
    // int y = idx % grid.cols();
    // return std::make_pair(x, y);
  };

  try {
    boost::astar_search_tree(
        g1, v0,
        distance_heuristic<Graph, Vertex,
                           std::remove_reference<decltype(GetXy)>::type, float>(
            GetXy, v1),
        boost::predecessor_map(prv_pmap)
            .distance_map(dist_pmap)
            // .weight_map(boost::get(&cho::graph::Edge2D::weight, g))
            .weight_map(wmap)
            .visitor(astar_goal_visitor<Vertex>(v1)));
  } catch (const found_goal& fg) {
    std::list<Eigen::Vector2i> path;
    auto v = v1;
    while (true) {
      int x, y;
      std::tie(x, y) = GetXy(v);
      // path.emplace_front(g[v].x, g[v].y);
      path.emplace_front(x, y);
      // fmt::print("{} {}", x, y);

      const auto& prv = prv_map[v];
      if (v == prv) {
        break;
      }
      v = prv;
    }

    return VecVector2i{path.begin(), path.end()};
  }
  return {};
}
}  // namespace graph
}  // namespace cho
