# FAST-LIO-ROS2
> Maintainer: Yunlong Feng  
> Modifier: Taeyoung Kim

## What's New?  
- Dockerize
- Step-by-step tutorial
- Add save trajectory function
- Add save map function without service callback

## Results
I currently only experiment after converting the ros2 bag format from the M2DGR dataset.  

           
https://github.com/Taeyoung96/FAST_LIO_ROS2/assets/41863759/27393bf1-740b-4347-ba8f-4c2170e50e11



<p align="center"><img src="https://github.com/Taeyoung96/FAST_LIO_ROS2/assets/41863759/c797fb90-8a10-47b2-af8e-f5bc87e3cc89" width = "500" ></p>  
<p align="center">< FAST-LIO2-ros2 trajectory result in Gate 01 sequence ></p>  


## How to use
**I recommend utilizing docker to easily configure your environment setting.**  

### Requirements 
- [Docker](https://www.docker.com/)  
- [NVIDIA docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

```
git clone --recursive https://github.com/Taeyoung96/FAST_LIO_ROS2.git
```

### Make a docker image (For the docker user) 
```
cd docker
```
```
docker build -t fast-lio-ros2:latest .
```

### Environment setting  
- ROS2 humble
- PCL
- Eigen
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

### Make a docker container  

When you create a docker container, you need several options to use the GUI and share folders.

First, you should enter the command below in the local terminal to enable docker to communicate with Xserver on the host.

```
xhost +local:docker
```

After that, make your own container with the command below.  

```
sudo chmod -R 777 container_run.sh
```
```
./container_run.sh <container_name> <image_name:tag>
```

:warning: **You should change {container_name}, {docker image} to suit your environment.**  

For example,  
```
 ./container_run.sh fast-lio-ros2 fast-lio-ros2:latest
```
In my case, I also shared an additional `/data` folder to run the ROS2 bag files.  

If you have successfully created the docker container, the terminal output will be similar to the below.  

**output** :  

```
Current working directory: /home/taeyoung/Desktop/FAST_LIO_ROS2
================FAST-LIO ROS2 Docker Env Ready================
root@taeyoung-cilab:~/ros2_ws#
```

### Build and Run the launch file

Within Docker, we need 2 containers.  
To access the same container, use the command below
```
docker exec -it fast-lio-ros2 /bin/bash
```  
Within the connected container,
```
source /opt/ros/humble/setup.bash
```

**This is an example for running M2DGR dataset.**
You need to use [rosbags](https://pypi.org/project/rosbags/) to change the ros1 bagfile to a ros2 bagfile already.  

```
root@taeyoung-cilab:~/ros2_ws# 
```

All should stay on the above path.

- **1st container** (For build fast_lio and livox_ros_driver2 package) 
```
cd src/livox_ros_driver2/
``` 
```
./build.sh humble
```
This process only needs to be done once, after connecting to the container.  
For all subsequent builds, simply proceed to `colcon build` on `ros2_ws`.

```
cd ~/ros2_ws/
```
```
source install/setup.bash 
```


For launch fast_lio ros2 version for M2DGR.  
```
ros2 launch fast_lio mapping_m2dgr.launch.py
```

- **2nd container** (For ros2 bag play) 
```
ros2 bag play gate_01
```


## Convert ROS1 bag to ROS2 bag  

- Use [rosbags](https://pypi.org/project/rosbags/) in docker container.  

## TODO  
- [ ] Test with more bag files.  
- [ ] Troubleshoot mapping speed issues

## Acknowledgments
Thanks to [FAST-LIO](https://github.com/hku-mars/FAST_LIO) for sharing their awesome research.  
Thanks also to [Ericsii](https://github.com/Ericsii/FAST_LIO) for providing the ros2 branch.  
I follow the license of each repository.  
