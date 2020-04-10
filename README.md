# MS-RPA 2020

This repository contains code for the 2020 ROS/Gazebo simulations of the MS-RPA algorithm.

## Instructions

This code was run on Ubuntu 18.04 with ROS Melodic, Gazebo v9.0, RViz v1.13.7. 

* Install `ros-melodic-full` on your system. Instructions can be found [here](http://wiki.ros.org/melodic/Installation/Ubuntu).
* Create a catkin workspace and clone this repo and the following git repos into the `src` folder:
    * [aion\_r1\_description](https://github.com/aionrobotics/aion_r1_description)
    * [aion\_r1\_gazebo](https://github.com/aionrobotics/aion_r1_gazebo)
    * [geographic\_info](https://github.com/ros-geographic-info/geographic_info) (may be optional if `ros-melodic-full` installed)
    * [hector\_gazebo](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo)
    * [hector\_localization](https://github.com/tu-darmstadt-ros-pkg/hector_localization)
    * [hector\_models](https://github.com/tu-darmstadt-ros-pkg/hector_models)
    * [hector\_quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor)
* Build your workspace using `catkin build`. Some hacking may be required. Welcome to ROS. `¯\_(ツ)_/¯`

This workspace configuration worked as of April 2020. Please note that the other packages listed above were only suppported through ROS Kinetic, and may possibly break when used with future versions of ROS. 


## License

This work is licensed under the MIT License.

