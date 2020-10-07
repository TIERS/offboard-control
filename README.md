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

