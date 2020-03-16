// Copyright [2020] Yoonyoung (Jamie) Cho

#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace cho {

using MatrixXu = Eigen::Matrix<std::uint8_t, Eigen::Dynamic, Eigen::Dynamic>;
using Vector2i = Eigen::Vector2i;
using VecVector2i = std::vector<Vector2i>;

namespace graph {
VecVector2i FindPath(const MatrixXu& grid, const Vector2i& iv0,
                     const Vector2i& iv1);
}

}  // namespace cho
