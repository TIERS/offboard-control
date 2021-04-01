[![Build Status](https://travis-ci.com/TIERS/offboard-control.svg?branch=main)](https://travis-ci.com/TIERS/offboard-control)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Offboard control
# PX4+MAVROS with UWB+VIO localization

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




## PC_Filter node

This node to filter a point cloud via a pass through filter and catch the center of our UGV through the green color in the center of you robot using a Realsense L515 lidar camera.



### Parameters
- `xpassthrough/filter_limit_min`: minimum x, anything below in x is filtered out
- `xpassthrough/filter_limit_max`: max x, anything above in x is filtered out
- `ypassthrough/filter_limit_min`: minimum y, anything below in y is filtered out
- `ypassthrough/filter_limit_max`: max y, anything above in y is filtered out
- `zpassthrough/filter_limit_min`: minimum z, anything below in z is filtered out
- `zpassthrough/filter_limit_max`: max z, anything above in z is filtered out
- `observed_frame_id`: This is the frame of reference for the original unfiltered point cloud
- `filtered_frame_id`: This is the frame of reference that the filtered point cloud will be rebroadcast in
- `input_pc_topic`: This is the ROS topic the unfiltered point cloud is being published on.
- `output_pc_topic`: This is the ROS topic in which the filtered point cloud will be published.
- `ugv_center_xy_topic`: This a the ROS topic in which the center of the UGV will be published.

### Usage
Place the following in a launch file:
```
<launch>
   <group ns="pc_filter">
      <param name="xpassthrough/filter_limit_min" value="-0.5" />
      <param name="ypassthrough/filter_limit_min" value="-0.5" />
      <param name="zpassthrough/filter_limit_min" value="0.2" />
      <param name="xpassthrough/filter_limit_max" value="0.5" />
      <param name="ypassthrough/filter_limit_max" value="0.5" />
      <param name="zpassthrough/filter_limit_max" value="1.0" />
      
      <param name="observed_frame_id" value="camera_depth_optical_frame" />
      <param name="filtered_frame_id" value="camera_depth_optical_frame" />
      <param name="input_pc_topic" value="/camera/depth/color/points" />
      <param name="output_pc_topic" value="/filtered_pc" />
      <param name="ugv_center_xy_topic" value="/ugv_center" />

      <node name="rs_pc_filter" pkg="pc_filter" type="pc_filter" output="screen"/>
   </group>
</launch>
```


## Contact

Visit us at [https://tiers.utu.fi](https://tiers.utu.fi)
