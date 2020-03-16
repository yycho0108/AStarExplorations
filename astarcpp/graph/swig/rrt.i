%module rrt

%include <stdint.i>
%include <std_vector.i>

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "bglpy/types.hpp"
#include "bglpy/sdf.hpp"
#include "bglpy/rrt.hpp"

using namespace cho::graph;
%}

%include <typemaps.i>
%include "numpy.i"

#include "bglpy/types.hpp"
%include "bglpy/sdf.hpp"
%include "bglpy/rrt.hpp"

using namespace cho::graph;
%template(NodeVector) ::std::vector<Node2D>;
%template(SdfVector) ::std::vector<SdfFun>;
