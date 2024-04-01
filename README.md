# AutoRace2024_public

## Installation and Setup
## Install AutoRace2024_public
* Replace [OWN_WORKSPACE] with your custom workspace name
```
$ mkdir [OWN_WORKSPACE]_ws && cd [OWN_WORKSPACE]_ws
$ git clone --recursive https://github.com/AndersonYu7/AutoRace2024_public src
```
## Build and Run Docker
* device -> pc/jetson
```
$ cd src/autorace_docker/autorace_docker_{device}
$ ./build.sh
$ ./run.sh
```
## Build your workspace
* build in your docker terminal
```
$ colcon build
$ source install/setup.bash
```

# Use H65_Camera
## Setup and Confirm your com
`$ ls /dev/video*`

* Determine if your interface is /dev/video2
* If not change line 23 of H65_camera.py to your interface

## Open Camera
`$ ros2 run camara H65_camera`

## Camera View
```
$ rviz2
add topic: /rgb image
```
