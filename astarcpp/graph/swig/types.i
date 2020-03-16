%module types

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "../include/types.hpp"
using namespace cho::graph;
%}

%include "../include/types.hpp"
