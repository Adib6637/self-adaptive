#!/bin/bash
clear

rm -rf build
mkdir build
cd build
sudo cmake ..
cd ..
./run.sh