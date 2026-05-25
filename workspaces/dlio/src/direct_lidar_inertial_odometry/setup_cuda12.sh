#!/bin/bash

# Setup script to ensure CUDA 12.6 is used
# Source this file before building or running DLIO

export PATH=/usr/local/cuda-12.6/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-12.6

echo "CUDA 12.6 environment configured"
echo "CUDA Version: $(nvcc --version | grep 'release' | awk '{print $6}')"
echo ""
echo "To build: ./build_gpu.sh"
echo "To run:   source install/setup.bash && ros2 launch direct_lidar_inertial_odometry dlio_mapping.launch.py"
