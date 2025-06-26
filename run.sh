#!/bin/bash
clear

rm -f log/log* 

cd build || { echo "Failed to change directory"; exit 1; }

 

if ! make; then
  echo "Make failed. Exiting."
  exit 1
fi

./simulation