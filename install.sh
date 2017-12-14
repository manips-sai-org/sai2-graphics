#!/bin/bash

git submodule update --init
cd chai3d
mkdir build
cmake .. && make -j8
cd ../..

 mkdir build
 cd build
 cmake .. && make -j8
 cd ..