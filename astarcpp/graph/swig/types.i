%module types

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "bglpy/types.hpp"
using namespace cho::graph;
%}

%include "bglpy/types.hpp"
