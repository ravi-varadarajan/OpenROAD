#!/bin/bash

script_path=`realpath $0`
topdir=`dirname ${script_path}`

#Assume build is done in build dir
build_dir="build"
topdir=${topdir}/${build_dir}
export PYTHONPATH="${topdir}/src/pygui/src/swig/python:${topdir}/src/pygui:${topdir}/src/OpenDB/src/swig/python"

echo ${PYTHONPATH}
${topdir}/src/openroad $*

