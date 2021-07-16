%module (directors="1") openroadpy
#pragma SWIG nowarn=473
%module openroadpy

%feature("director:except") {
    if( $error != NULL ) {
        PyObject *ptype, *pvalue, *ptraceback;
        PyErr_Fetch( &ptype, &pvalue, &ptraceback );
        PyErr_Restore( ptype, pvalue, ptraceback );
        PyErr_Print();
        Py_Exit(1);
    }
} 

%{
using namespace std ;

#define SWIG_FILE_WITH_INIT
#include "openroadPyIntf.h"
#include "openroadGeom.h"
#include "openroadUiEnums.h"
#include "openroadLayer.h"
#include "openroadShape.h"
#include "openroadCanvas.h"
#include "openroadView.h"
#include "openroadTransform.h"
#include "openroadMotion.h"
using namespace OpenRoadUI ;
%}

%feature ("director") OpenRoadPythonIntf ;

%include "typemaps.i"
%include "std_pair.i"
%include "std_string.i"
%include "std_vector.i"
%include "std_set.i"
%include "stl.i"

%include "openroadPyIntf.h"
%include "openroadGeom.h"
%include "openroadUiEnums.h"
%include "openroadLayer.h"
%include "openroadShape.h"
%include "openroadCanvas.h"
%include "openroadTransform.h"
%include "openroadMotion.h"
%include "openroadView.h"
%inline %{
//typedef unsigned int size_t;
typedef unsigned int uint ;
%}

%typemap(in) (uint) = (int);
%typemap(out) (uint) = (int);
%typemap(out) (uint64) = (long);
%apply double& INOUT { double& a };

%typemap(typecheck,precedence=SWIG_TYPECHECK_INTEGER) uint {
  $1 = PyInt_Check($input) ? 1 : 0;
}

using namespace std ;
using namespace OpenRoadUI ;

%{
#include <vector>
#include <string>
#include <set>
#include <map>
#include <tuple>
%}

%apply const char* {std::string &x};
%apply char* {std::string &x};
%apply const std::string& {std::string* foo};

namespace std {
    %template(vector_string)                vector<std::string> ;
    %template(pair_uint_shape)              pair<uint, GLShape*> ;
    %template(vector_uint)                  vector<uint> ;
} ;
