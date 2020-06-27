#!/usr/bin/env bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
cd ..
mv build/region.so .
# rm -rf build
