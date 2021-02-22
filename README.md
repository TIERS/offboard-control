# Offboard control: PX4+MAVROS with UWB+VIO localization

Offboard controller for PX4 flight controllers with MAVROS. Uses UWB+VIO for localizartion in GNSS-denied environments.

This package is compatible with 

## Installation

Install mavros
```
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
```

## Installation of Simulation Environment

Full instructions are available here: [https://dev.px4.io/master/en/simulation/ros_interface.html](https://dev.px4.io/master/en/simulation/ros_interface.html)


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

To run a simple example, you can use the `safe_offboard` script which by default hovers the drone 1.2m over its starting position. It uses UWB for positioning (tfmini lidar for `z`) and VIO for orientation. Does not use by dedault VIO to smooth the UWB positioning.

To start the `dwm1001` interface, `tf-mini` and `mavros`, run:
```
roslaunch offboard_control uwb_lidar_pos.launch
```

then start the T265 camera
```
roslaunch offboard_control t265.launch
```

and optionally the RealSense D435 with
```
roslaunch offboard_control d435.launch
```

You will need to specify the `serial_no` paramerer (serial number) in both cases. Alternatively, the T265 can be launched with
```
roslaunch realsense2_camera rs_t265.launch
```

The serial numbers can be obtained by connecting one camera at a time and launching
```
roslaunch realsense2_camera rs_camera.launch
```
which will automatically detect the camera and output the corresponding serial number.


## Contact

For any questions, write to `jopequ@utu.fi`.

Visit us at [https://tiers.utu.fi](https://tiers.utu.fi)
