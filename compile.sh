#!/bin/bash
##############################################################################
# 
# v1.0 [William Arias] - initial version
#
##############################################################################

. /opt/$ROS_DISTRO/setup.bash

rm -rf ./build
cmake -S . -B ./build -DCMAKE_INSTALL_PREFIX=./install
cmake --build ./build
cmake --install ./build
