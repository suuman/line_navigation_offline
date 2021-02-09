#!/bin/bash
cd linenav/arpack++
make clean
make all
cd ../..
mkdir build
cd build
cmake ..
make -j8

