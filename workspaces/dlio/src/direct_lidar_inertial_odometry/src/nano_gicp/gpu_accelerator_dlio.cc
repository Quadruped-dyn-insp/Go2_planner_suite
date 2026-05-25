/***********************************************************
 * CUDA Integration - DLIO Point Type Template Instantiation
 ***********************************************************/

// Include the implementation to make templates available
#include "gpu_accelerator.cc"
#include "dlio/dlio.h"

namespace nano_gicp {

// Explicit template instantiation for DLIO Point type
template void GpuAccelerator::cloudToArray<dlio::Point>(
    const pcl::PointCloud<dlio::Point>::ConstPtr&, std::vector<float>&);
    
template void GpuAccelerator::arrayToCloud<dlio::Point>(
    const std::vector<float>&, pcl::PointCloud<dlio::Point>::Ptr&);
    
template bool GpuAccelerator::calculateCovariances<dlio::Point>(
    const pcl::PointCloud<dlio::Point>::ConstPtr&, int,
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>&, float&);
    
template void GpuAccelerator::transformPointCloud<dlio::Point>(
    const pcl::PointCloud<dlio::Point>::ConstPtr&,
    pcl::PointCloud<dlio::Point>::Ptr&,
    const Eigen::Matrix4f&);

}  // namespace nano_gicp
