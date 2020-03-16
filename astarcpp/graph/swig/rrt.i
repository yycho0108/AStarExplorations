%module rrt

%include <stdint.i>
%include <std_vector.i>

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "../include/types.hpp"
#include "../include/sdf.hpp"
#include "../include/rrt.hpp"

using namespace cho::graph;
%}

%include <typemaps.i>
%include "numpy.i"

#include "../include/types.hpp"
%include "../include/sdf.hpp"
%include "../include/rrt.hpp"

using namespace cho::graph;
%template(NodeVector) ::std::vector<Node2D>;
%template(SdfVector) ::std::vector<SdfFun>;
