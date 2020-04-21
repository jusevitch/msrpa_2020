# MS-RPA 2020

This repository contains code for the 2020 ROS/Gazebo simulations for swarm of the MS-RPA algorithm.

[![Project Status: WIP – Initial development is in progress, but there has not yet been a stable, usable release suitable for the public.](https://www.repostatus.org/badges/latest/wip.svg)](https://www.repostatus.org/#wip)


## Installation and Requirements

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
* Ensure that the following packages **for Python 2.7** are installed (by using pip):
    * Numpy / Scipy (`pip2 install numpy scipy`)
    * [OSQP](https://osqp.org/) (`pip2 install osqp`)

This workspace configuration worked as of April 2020. Please note that the other packages listed above were only suppported through ROS Kinetic, and may possibly break when used with future versions of ROS. 

### Optional Tools

These tools are not required to use this package, but may make your life easier.

* [EasyLaunch](https://github.com/jusevitch/easylaunch): A Python module that lets you script launch files in Python, then generates the XML launch file automatically.

## Quick Demos

To see it in action, source your ROS workspace and run one of the launch files from the following list:

* `launch/easylaunch_scripts/one_rover_2_cubes.launch`: One R1 rover tracks a circular trajectory while avoiding two cube obstacles. A little boring, but it works.

## Instructions

_Coming soon..._

## License

This work is licensed under the MIT License.

