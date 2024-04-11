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

# Execution
## Use H65_Camera
### Setup and Confirm your com
`$ ls /dev/video*`

* Determine if your interface is /dev/video2
  - If not change H65_camera.py to your interface

### Open Camera
`$ ros2 run camara H65_camera`

### Camera View
```
$ rviz2
add topic: /image/image_raw image
```

## Control Motor
### Setup and Confirm your device
`$ ls /dev/ttyUSB*`

* Determine if your interface is /dev/ttyUSB0
  - If not change control_dual_{model}.py.py to your interface
* model -> xl/xm

### Trun on control motor
```
$ sudo chmod 777 /dev/ttyUSB*
$ ros2 launch control_motor control_dual_{model}_launch.py
```
* model -> xl/xm

### Keyboard control
`$ ros2 run teleop_twist_keyboard teleop_twist_keyboard`

## Module
* [Detect Lane](/detect/detect_lane/README.md)
* [Object Detct](/detect/object_detection/README.md)
