#!/bin/bash
clear

rm -rf build
mkdir build
cd build

# Use environment variables or fail with a clear message
echo "SYSTEMC_HOME: $SYSTEMC_HOME"
echo "GUROBI_HOME: $GUROBI_HOME"
if [[ -z "$SYSTEMC_HOME" ]]; then
    echo "Error: SYSTEMC_HOME is not set. Please export SYSTEMC_HOME or set it before running this script."
    exit 1
fi
if [[ -z "$GUROBI_HOME" ]]; then
    echo "Error: GUROBI_HOME is not set. Please export GUROBI_HOME or set it before running this script."
    exit 1
fi

cmake .. -DSYSTEMC_HOME=$SYSTEMC_HOME -DGUROBI_HOME=$GUROBI_HOME
cd ..
./run.sh