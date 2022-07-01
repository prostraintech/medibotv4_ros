
# Medibot V4   [![Badge License]][License]

*ROS packages and firmware code.*

<br>

## Requirements

### Software

- Ubuntu 18.04
- ROS Melodic Morenia

<br>

### Hardware

- Advantech MIC-710AIX / Jetson Xavier NX (robot computer)

- Arduino Due (base controller)

- Hokuyo UST-05LA (lidar)

- 2 x Motors with Encoders

<br>
<br>

## Installation

1.  Clone the repository

    ```shell
    cd ~/catkin_ws/src &&
    git clone https://github.com/zulhafiz-zulkifli/medibotv4_ros.git
    ```
    
    <br>
    
2.  Install packages dependencies 

    ```shell
    rosdep install medibotv4 -y
    ```
    
    <br>

3. Build the packages 
    
    ```shell
    cd ~/catkin_ws &&
    catkin_make &&
    source ~/catkin_ws/devel/setup.bash
    ```
    
<br>
<br>

## Simulation

### Single Robot

```shell
roslaunch medibotv4 simulation.launch
```

<br>

### Multi Robot

```shell
roslaunch medibotv4 multirobot_simulation.launch
```

<br>


<!----------------------------------------------------------------------------->

[Badge License]: https://img.shields.io/badge/License-Unknown-808080.svg?style=for-the-badge

[License]: #
