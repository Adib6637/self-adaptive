#!/bin/bash
clear

rm -f log/log* 
rm -f log/*.png
rm -f weight/* 

cd build || { echo "Failed to change directory"; exit 1; }

 

if ! sudo -E make; then
  echo "Make failed. Exiting."
  exit 1
fi

# ./simulation