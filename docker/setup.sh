#!/bin/bash

set -xe

NUM_CORES=`getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu || echo 1`

NUM_PARALLEL_BUILDS=$((NUM_CORES-2))
if [ $NUM_PARALLEL_BUILDS -lt 1 ]; then
    NUM_PARALLEL_BUILDS=1
fi

cd /root/catkin_ws/src 
# Check if .rosinstall file exists (indicating wstool init has been run)
if [ ! -f .rosinstall ]; then
  wstool init
fi

# Check if depend_pack.rosinstall has been merged (check if it's in the .rosinstall file)
if ! grep -q "depend_pack.rosinstall" .rosinstall; then
  wstool merge /root/catkin_ws/src/OA-LICalib/depend_pack.rosinstall
fi
wstool update

# Install the ceres-solver and pangolin packages
cd /opt/local/lib/
mkdir -p ceres-solver/build
pushd ceres-solver/build 
cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$NUM_PARALLEL_BUILDS && make install
popd

# Make Pangolin
mkdir -p Pangolin/build 
pushd Pangolin/build
cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$NUM_PARALLEL_BUILDS
popd

# Make Sophus
mkdir -p Sophus/build 
pushd Sophus/build
cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$NUM_PARALLEL_BUILDS
popd


# Prepare ndt_omp separately
cd /root/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="ndt_omp"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
