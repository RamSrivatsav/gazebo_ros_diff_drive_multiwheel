# Gazebo ROS differential Drive Plugin (Multiwheel)
This plugin solves the drawback of inbuilt differential drive plugin which supports only two wheels.
This plugin is developed to support 2N number of wheels, where N is a natural number.

**Note:** This plugin is an extension of the inbuilt differential drive plugin.

## Requirements:
Here are the requirements of the plugin.
* ROS Melodic
* Gazebo

## Features:
* Publishes the TF and Joint state messages.
* Publishes the Odom topic.

## Installation
* Clonning the plugin repository into your catkin workspace.
```bash
cd catkin_ws/src
git clone https://github.com/RamSrivatsav/gazebo_ros_diff_drive_multiwheel.git
```
* Compiling the repository
```bash
cd ../..
catkin_make
```

## Testing the plugin
* Here is a brief clip of the plugin in action.
![](output_gif.gif)