#!/bin/bash
# Script to replace std::vector with raw arrays in CUDA files

FILE="src/nano_gicp/cuda/cuda_gicp.cu"

# Replace std::vector declarations
sed -i 's/std::vector<GpuPoint> h_source(num_source), h_target(num_target);/GpuPoint* h_source = new GpuPoint[num_source];\n    GpuPoint* h_target = new GpuPoint[num_target];/' "$FILE"
sed -i 's/std::vector<float> transformed_source(num_source \* 3);/float* transformed_source = new float[num_source * 3];/' "$FILE"
sed -i 's/std::vector<float> target_flat(num_target \* 3);/float* target_flat = new float[num_target * 3];/' "$FILE"
sed -i 's/std::vector<int> knn_indices(num_source);/int* knn_indices = new int[num_source];/' "$FILE"
sed -i 's/std::vector<float> knn_sq_dists(num_source);/float* knn_sq_dists = new float[num_source];/' "$FILE"
sed -i 's/std::vector<double> h_H_blocks(num_source \* 36);/double* h_H_blocks = new double[num_source * 36];/' "$FILE"
sed -i 's/std::vector<double> h_b_blocks(num_source \* 6);/double* h_b_blocks = new double[num_source * 6];/' "$FILE"
sed -i 's/std::vector<double> h_errors(num_source);/double* h_errors = new double[num_source];/' "$FILE"

echo "Fixed std::vector declarations in $FILE"
