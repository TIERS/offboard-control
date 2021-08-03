
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# PX4+MAVROS with UWB+VIO relative MRS localization


| Status  |
|---------|
| ![ROS Melodic build badge](https://github.com/TIERS/offboard-control/actions/workflows/melodic.yml/badge.svg) |


## Description

Offboard controller for PX4 flight controllers with MAVROS. Uses UWB+VIO for localizartion in GNSS-denied environments.

## Installation

Clone this repo to your `catking_ws/src/`
```
git clone https://github.com/TIERS/offboard-control.git
```

Install mavros
```
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
```

Install geographiclib
```
sudo apt install geographiclib-tools -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

Build the package
```
catkin build
```

## Main ROS nodes and functionalities

`TO DO`


## Contact

Visit us at [https://tiers.utu.fi](https://tiers.utu.fi)
