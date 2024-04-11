# Detect Lane

## Prerequisites

Make sure you have Docker installed on your system. If not, follow the Docker installation instructions for your platform.

* [AutoRace Installation](README.md#installation-and-setup)

## Execution
### Step 1: Launch the detect program 
`$ ros2 launch detect_lane detect_lane_launch.py calibration:=False`

#### Parameter Description
* **calibration**:
  - True: Enable calibration mode to adjust HSV and image parameters for optimal lane detection performance.
  - False: Enable line following mode, which outputs the middle line for navigation.

### Step 2: Open Motor Drive

* [Control_Motor](README.md#Trun-on-control-motor)

### Step 3: Run Control Lane program 
`$ ros2 run control control_lane`

## Challenge
At the start, the program fails to detect the lane and throws errors.
