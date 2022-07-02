
# IIUM Medibot V4   [![Badge License]][License]

*ROS packages and firmware code.*


## Requirements

### Software

- Ubuntu 18.04
- ROS Melodic Morenia

### Hardware

- Advantech MIC-710AIX / Jetson Xavier NX (robot computer)
- Arduino Due (base controller)
- Hokuyo UST-05LA (lidar)
- 2 x Motors with Encoders

## Installation

1.  Clone the repository
    ```shell
    cd ~/catkin_ws/src &&
    git clone https://github.com/zulhafiz-zulkifli/medibotv4_ros.git
    ```

2.  Install packages dependencies 
    ```shell
    rosdep install medibotv4 -y
    ```

3. Build the packages 
    ```shell
    cd ~/catkin_ws &&
    catkin_make &&
    source ~/catkin_ws/devel/setup.bash
    ```

## How to Use

### Robot bringup (Base controller and hardware interface
```shell
roslaunch medibotv4 bringup.launch
```

### Mapping using SLAM
```shell
roslaunch medibotv4 gmapping.launch
```

### Autonomous Navigation
```shell
roslaunch medibotv4 navigation.launch
```

### Simulation

- Single Robot
```shell
roslaunch medibotv4 simulation.launch
```

- Multi Robot
```shell
roslaunch medibotv4 multirobot_simulation.launch
```

<!----------------------------------------------------------------------------->

[Badge License]: https://img.shields.io/badge/License-BSD_3--Clause-blue.svg?style=for-the-badge
[License]: LICENSE
