###########################################################
#                                                         #
# Copyright (c)                                           #
#                                                         #
# The Verifiable & Control-Theoretic Robotics (VECTR) Lab #
# University of California, Los Angeles                   #
#                                                         #
# Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   #
# Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         #
#                                                         #
###########################################################

cmake_minimum_required(VERSION 3.12.4)
project(direct_lidar_inertial_odometry)

# Option to enable/disable GPU acceleration (default OFF due to CUDA/GCC compatibility)
option(ENABLE_GPU "Enable GPU acceleration (requires compatible CUDA/GCC)" OFF)

# Enable C++ language
enable_language(CXX)

# Conditionally enable CUDA - IMPORTANT: Set CMAKE_CUDA_HOST_COMPILER BEFORE enable_language(CUDA)
if(ENABLE_GPU)
  # Use GCC 10 as CUDA host compiler to avoid std::function compatibility issues with GCC 11
  set(CMAKE_CUDA_HOST_COMPILER "/usr/bin/gcc-10")
  set(CMAKE_CUDA_ARCHITECTURES 75)
  message(STATUS "Using CUDA host compiler: /usr/bin/gcc-10")
  enable_language(CUDA)
  message(STATUS "GPU acceleration ENABLED")
else()
  message(STATUS "GPU acceleration DISABLED (CPU-only build)")
  add_compile_definitions(DISABLE_GPU_ACCELERATION)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_srvs REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

if(ENABLE_GPU)
  set(CMAKE_CUDA_STANDARD 17)
  
  # CUDA compilation flags
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr --extended-lambda")
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -fPIC")
  set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} -O3 --use_fast_math")
endif()

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Generate ROS interfaces
rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/SavePCD.srv
    DEPENDENCIES       
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(SYSTEM)

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_gicp/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
ament_target_dependencies(nanoflann)

if(ENABLE_GPU)
  # CUDA-accelerated modules
  add_library(cuda_modules STATIC
    src/nano_gicp/cuda/cuda_knn.cu
    src/nano_gicp/cuda/cuda_covariance.cu
    src/nano_gicp/cuda/cuda_transform.cu
    src/nano_gicp/cuda/cuda_gicp.cu
  )
  set_target_properties(cuda_modules PROPERTIES 
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    POSITION_INDEPENDENT_CODE ON
    CUDA_STANDARD 17
  )
  target_include_directories(cuda_modules PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(cuda_modules ${PCL_LIBRARIES})
  
  # NanoGICP with GPU
  add_library(nano_gicp STATIC
    src/nano_gicp/lsq_registration.cc
    src/nano_gicp/nano_gicp.cc
    src/nano_gicp/gpu_accelerator.cc
  )
  target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann cuda_modules)
  ament_target_dependencies(nano_gicp)
  
  # Odometry Node with GPU
  add_executable(dlio_odom_node src/dlio/odom_node.cc src/dlio/odom.cc src/nano_gicp/gpu_accelerator_dlio.cc)
  target_link_libraries(dlio_odom_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp cuda_modules)
else()
  # NanoGICP CPU-only (without gpu_accelerator.cc)
  add_library(nano_gicp STATIC
    src/nano_gicp/lsq_registration.cc
    src/nano_gicp/nano_gicp.cc
  )
  target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
  ament_target_dependencies(nano_gicp)
  
  # Odometry Node CPU-only
  add_executable(dlio_odom_node src/dlio/odom_node.cc src/dlio/odom.cc)
  target_link_libraries(dlio_odom_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp)
endif()

ament_target_dependencies(dlio_odom_node "tf2_ros" "pcl_ros" "nav_msgs" "pcl_conversions" "rclcpp" "sensor_msgs" "geometry_msgs")

# Mapping Node
add_executable(dlio_map_node src/dlio/map_node.cc src/dlio/map.cc)
target_link_libraries(dlio_map_node ${PCL_LIBRARIES} ${cpp_typesupport_target})
ament_target_dependencies(dlio_map_node "tf2_ros" "pcl_ros" "pcl_conversions" "rclcpp")

ament_export_include_directories(include)
if(ENABLE_GPU)
  ament_export_libraries(${PROJECT_NAME} nano_gicp nanoflann cuda_modules)
else()
  ament_export_libraries(${PROJECT_NAME} nano_gicp nanoflann)
endif()
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros)
ament_package()

# Binaries
install(TARGETS dlio_odom_node dlio_map_node
        DESTINATION lib/${PROJECT_NAME} 
)
install(DIRECTORY cfg launch
        DESTINATION share/${PROJECT_NAME} 
)

# Install Python manager node
install(PROGRAMS src/dlio/dlio_manager.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME dlio_manager
)

# Compiler optimization flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

link_directories(${PCL_LIBRARY_DIRS})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

target_compile_options(dlio_odom_node PRIVATE ${OpenMP_FLAGS})
target_compile_options(dlio_map_node PRIVATE ${OpenMP_FLAGS})
