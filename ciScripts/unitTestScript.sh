#!/bin/bash

set -ev

mkdir -p build
cd build
cmake -BUILD_TESTS=ON ../
make -j2
make test
