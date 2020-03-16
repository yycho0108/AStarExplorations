%module astar

%include <std_vector.i>

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include <Eigen/Core>
#include "bglpy/graph.hpp"
%}

%include <typemaps.i>
%include "numpy.i"
%include "eigen.i"

%eigen_typemaps(Eigen::Vector2i)
%eigen_typemaps(Eigen::Matrix<std::uint8_t, Eigen::Dynamic, Eigen::Dynamic>)

%include "bglpy/graph.hpp"

%template(IntVector) ::std::vector<int>;
%template(VecVector2i) ::std::vector<Eigen::Vector2i>;
