# Offboard control: PX4+MAVROS with UWB+VIO localization

Offboard controller for PX4 flight controllers with MAVROS. Uses UWB+VIO for localizartion in GNSS-denied environments.

This package is compatible with 

## Installation

Install mavros
```
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
```

## Installation of Simulation Environment

Full instructions are available here: (https://dev.px4.io/master/en/simulation/ros_interface.html)[https://dev.px4.io/master/en/simulation/ros_interface.html]


Install geographiclib
```
sudo apt install geographiclib-tools -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```





## Run Simulation

Run the Gazebo simulator with PX4 SITL:
```
cd ~/offboard_ws/PX4Firmware
```

To connect mavros to the PX4 SITL simulator, run
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

## Run using UWB (VIO optional) for localization

To run a simple example, you can use the `safe_offboard` script which by default hovers the drone 1.2m over its starting position. It uses UWB for positioning (tfmini lidar for z) and VIO for orientation. Does not use by dedault VIO to smooth the UWB positioning. Running the px4 and realsense t265 launch files from within the same launch file causes problems with the connections to the rostore. To avoid that, first open a terminal and start a roscore:

```
source ~/offboard_ws/devel/setup.bash
roscore
```

and then in a second terminal

```
source ~/offboard_ws/devel/setup.bash
roslaunch offboard_control uwb_lidar_pos.launch
```
