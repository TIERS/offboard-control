#!/usr/bin/env python

import os
import sys
import math
import time
import rospy
import numpy

from std_msgs.msg       import String
from std_msgs.msg       import Float64
from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import Range
from geometry_msgs.msg  import PoseStamped

from offboard_control.srv  import flight_mode as OffMode

from .utils import relative_position_estimation

class LandingControl() :

    def __init__(self) :
        '''
            Test node to land a UAV on a moving UGV based on UWB ranging and VIO with common orientation frame.
            This node performs landing using a single UAV-UGV range measurement + VIO on both vehicles.
        '''

        # Init node
        rospy.init_node('UWB_VIO_Landing', anonymous=False)

        # Positions for navigation
        self.pos = PoseStamped()
        self.uav_uwb_pos = PoseStamped()
        self.ugv_uwb_pos = PoseStamped()
        self.objective = PoseStamped()
        self.max_pos_delay = rospy.get_param("~max_pos_delay", 0.2)
        self.waypoint_step = rospy.get_param("~waypoint_step", 0.2)

        # Control altitudes
        self.approaching_altitude = rospy.get_param("~approaching_altitude", 1.2)
        self.closing_altitude = rospy.get_param("~closing_altitude", 0.8)
        self.landing_altitude = rospy.get_param("~landing_altitude", 0.48)
        self.closing_dist = rospy.get_param("~closing_dist", 1)
        self.landing_dist = rospy.get_param("~landing_dist", 0.1)

        # Publish distance to target in a topic
        self.dist_to_target_topic = rospy.get_param("~dist_to_target_topic", "landing/distance")
        self.dist_to_target = -1
        self.dist_to_target_pub = rospy.Publisher(self.dist_to_target_topic, Range, queue_size=10)

        # Get pos from PX4 EKF2 that already integrates VIO and/or others
        self.local_pos_topic =  rospy.get_param("~local_pos_topic", "/uav/mavros/local_position/pose")
        rospy.loginfo("Subscribing to local (mavros) position at {}".format(self.local_pos_topic))
        self.local_pos_sub = rospy.Subscriber(self.local_pos_topic, PoseStamped, self.update_current_pos_cb)

        # Wait to get local pos lock
        sys.stdout.write("Waiting for (mavros) local pos lock...")
        sys.stdout.flush()
        waiting_cnt = 0.1
        while not self.pos.header.stamp :
            sys.stdout.write("\rWaiting for (mavros) local pos lock... {}s".format(waiting_cnt))
            sys.stdout.flush()
            waiting_cnt += 0.1
            time.sleep(0.1)
        self.home_pos = self.pos
        rospy.loginfo("Home position locked: \n{}".format(self.pos.pose))

        # Get offboard flight mode
        self.offboard_mode = "unkown"
        self.offboard_mode_topic = rospy.get_param("~offboard_mode_topic", "/uav/offboard/mode")
        rospy.loginfo("Subscribing to offboard mode at {}".format(self.offboard_mode_topic))
        self.offbord_state_sub = rospy.Subscriber(self.offboard_mode_topic, String, self.update_offboard_mode_cb)

        # Get offboard state
        self.offboard_state = "unknown"
        self.offboard_state_topic = rospy.get_param("~offboard_state_topic", "/uav/offboard/state")
        rospy.loginfo("Subscribing to offboard state at {}".format(self.offboard_state_topic))
        self.offbord_state_sub = rospy.Subscriber(self.offboard_state_topic, String, self.update_offboard_state_cb)

        # UGV VIO pose (lower freq TODO)
        self.ugv_vio_odom = Odometry()
        self.ugv_vio_topic = rospy.get_param("~ugv_vio_topic", "/ugv/t265/odom/sample")
        rospy.loginfo("Subscribing to UGV VIO odometry at {}".format(self.ugv_vio_topic))
        self.ugv_vio_subscriber = rospy.Subscriber(self.ugv_vio_topic, Odometry, self.ugv_vio_cb)

        # Publisher for actual control
        self.waypoint_pub_topic = rospy.get_param('~waypoint_pub_topic', "/dwm1001/tag/dronie/position")
        self.control_pub = rospy.Publisher(self.waypoint_pub_topic, PoseStamped, queue_size=10)

        # Service for changing to external offboard mode
        self.pub_state_mode = rospy.Publisher('/uav/offboard/mode/change', String, queue_size=10)
        self.set_ofboard_mode_client  = rospy.ServiceProxy("/uav/offboard/cmd/mode", OffMode)

        # Custom UWB ranging subscriber
        self.custom_uwb_range_sub = rospy.Subscriber("/dwm1001/tag_A/distance", Float64, self.custom_uwb_cb)
        self.custom_uwb_range = -1
        self.custom_uwb_stamp = rospy.Time()

        # Initial relative pos measurements
        self.min_accumulated_meas = rospy.get_param('~min_accumulated_meas', 20)
        self.min_intermeas_dist = rospy.get_param('~min_intermeas_dist', 0.3)
        self.accumulated_meas = []
        self.relative_pos_estimation_done = False


    def update_current_pos_cb(self, pos) :
        '''
            Update position from mavros local pose (callback)
        '''
        self.pos = pos

    def update_offboard_state_cb(self, state) :
        '''
            Update offboard state (callback)
        '''
        self.offboard_state = state.data

    def update_offboard_mode_cb(self, mode) :
        '''
            Update offboard mode (callback)
        '''
        self.offboard_mode = mode.data

    def ugv_vio_cb(self, odom) :
        '''
            Updata UGV VIO odom (callback)
        '''
        self.ugv_vio_odom = odom

    def custom_uwb_cb(self, range) :
        '''
            Update range to custom UWB A
        '''
        self.custom_uwb_range = range.data - 0.3
        if self.custom_uwb_range < 0 :
            self.custom_uwb_range = 0.1 # TODO check this
        self.custom_uwb_stamp = rospy.Time.now()

    def publish_dist_to_target(self, target) :
        '''
            Publish distance to target (timer callback)
        '''
        r = Range()
        r.header.stamp = rospy.Time.now()
        r.range = self.custom_uwb_range
        self.dist_to_target_pub.publish(r)

    def update_objective(self, target) :
        '''
            Update objective waypoint in order to move towards target and land (timer callback)
        '''
        self.objective = PoseStamped()

        if rospy.Time.now() - self.ugv_vio_odom.header.stamp > rospy.Duration(self.max_pos_delay) :
            rospy.logwarn("Not receiving UGV VIO pose updates...")
        elif rospy.Time.now() - self.custom_uwb_stamp > rospy.Duration(self.max_pos_delay) :
            rospy.logwarn("Not receiving UWB range updates...")
        elif rospy.Time.now() - self.pos.header.stamp > rospy.Duration(self.max_pos_delay) :
            rospy.logwarn("Not receiving MAVROS local POS update...")
        else :

            if len(self.accumulated_meas) < self.min_accumulated_meas and self.offboard_mode is "circling" :
                
                # Keep circling while taking ranging measurements
                # First take one if none taken yet
                if len(self.accumulated_meas) < 1 :
                    self.accumulated_meas.append(
                        [self.pos, self.ugv_vio_odom, self.custom_uwb_range]
                    )
                # Then check we have move enough distance from last meas
                else :
                    current_pos = numpy.array( [self.pos.pose.position.x, self.pos.pose.position.y] )
                    last_meas = self.accumulated_meas[-1]
                    last_pos = numpy.array( [last_meas[0].pos.pose.position.x, last_meas[0].pos.pose.position.y] )
                    dist_from_last_meas = numpy.linalg.norm(current_pos, last_pos)
                    if dist_from_last_meas > self.min_intermeas_dist :
                        self.accumulated_meas.append(
                            [self.pos, self.ugv_vio_odom, self.custom_uwb_range]
                        )

            else :

                # Reusing code from relative position mode
                ugv_pos = self.starting_ugv_pos + numpy.array([
                    self.ugv_uwb_pos.pose.position.x,
                    self.ugv_uwb_pos.pose.position.y
                ])
                uav_pos = numpy.array([
                    self.uav_uwb_pos.pose.position.x,
                    self.uav_uwb_pos.pose.position.y
                ])
                move_dir = ugv_pos - uav_pos
                self.dist_to_target = numpy.linalg.norm(move_dir)
                if self.dist_to_target > 0.05 :
                    move_dir = self.waypoint_step * move_dir / self.dist_to_target
                
                self.objective = self.pos
                self.objective.header.stamp = rospy.Time.now()
                self.objective.pose.position.x += move_dir[0]
                self.objective.pose.position.y += move_dir[1]

                if self.dist_to_target > self.closing_dist :
                    self.objective.pose.position.z = self.approaching_altitude
                elif self.dist_to_target > self.landing_dist :
                    self.objective.pose.position.z = self.closing_altitude
                elif self.dist_to_target <= self.landing_dist and self.dist_to_target > 0 :
                    self.objective.pose.position.z = self.landing_altitude
                else :
                    rospy.logerr("Distance to target of {} does not make sense...".format(self.dist_to_target))

                # Clone orientation to UGV's if available
                if rospy.Time.now() - self.ugv_vio_odom.header.stamp < rospy.Duration(self.max_pos_delay) :
                    self.objective.pose.orientation.x = self.ugv_vio_odom.pose.pose.orientation.x
                    self.objective.pose.orientation.y = self.ugv_vio_odom.pose.pose.orientation.y
                    self.objective.pose.orientation.z = self.ugv_vio_odom.pose.pose.orientation.z
                    self.objective.pose.orientation.w = self.ugv_vio_odom.pose.pose.orientation.w
                else :
                    self.objective.pose.orientation.x = 0
                    self.objective.pose.orientation.y = 0
                    self.objective.pose.orientation.z = 0
                    self.objective.pose.orientation.w = 1

                self.control_pub.publish(self.objective)

    def run(self) :
        '''
            Infinite loop that flies the drone
        '''

        # Set a rospy state check rate and landing check rate
        status_check_rate = rospy.Rate(10.0)

        # Set objective update timer
        time.sleep(1)
        self.objective_timer = rospy.Timer(rospy.Duration(0.1), self.update_objective)

        # Set distance to target publish timer
        self.dist_to_target_timer = rospy.Timer(rospy.Duration(0.1), self.publish_dist_to_target)

        # Set offboard mode to 'external_control'
        rospy.loginfo("Setting offboard mode to external...")
        rospy.wait_for_service('/uav/offboard/cmd/mode')
        try:
            mode = String()
            mode.data = "external_control"
            self.pub_state_mode.publish(mode)
            self.set_ofboard_mode_client("external_control")
        except rospy.ServiceException as e:
            print("Service failed: {}. Offboard mode cannot be changed to external control.".format(e))

        # Start flying towards target and check when we should land
        rospy.loginfo("Everything in order. Let's fly.")
        try:
            while not rospy.is_shutdown() :
                
                # First check if we should be flying
                if len(self.accumulated_meas) < self.min_accumulated_meas :
                    # Make sure we are in "circling" mode if we are already flying
                    if self.offboard_state is "flying" and self.offboard_mode is not "circling" :
                        rospy.loginfo("Setting offboard mode from '{}' to 'circling'...".format(self.offboard_mode))
                        rospy.wait_for_service('/uav/offboard/cmd/mode')
                        try:
                            mode = String()
                            mode.data = "circling"
                            self.pub_state_mode.publish(mode)
                            self.set_ofboard_mode_client("circling")
                        except rospy.ServiceException as e:
                            print("Service failed: {}. Offboard mode cannot be changed to circling.".format(e))

                # Then if all meas have been taken, estimate relative pose
                elif not self.relative_pos_estimation_done :
                    self.starting_ugv_pos = relative_position_estimation(self.accumulated_meas)

                # Finally go to external and start approaching UGV for landing
                elif rospy.Time.now() - self.objective.header.stamp < rospy.Duration(self.max_pos_delay):

                    if self.offboard_mode != "external_control" :
                        rospy.loginfo("Setting offboard mode from '{}' to 'external_control'...".format(self.offboard_mode))
                        rospy.wait_for_service('/uav/offboard/cmd/mode')
                        try:
                            mode = String()
                            mode.data = "external_control"
                            self.pub_state_mode.publish(mode)
                            self.set_ofboard_mode_client("external_control")
                        except rospy.ServiceException as e:
                            print("Service failed: {}. Offboard mode cannot be changed to external_control.".format(e))

                    else:
                        pass # TODO everything normal, wait to be on position to land

                else :
                    rospy.loginfo("Measurements taken, waiting for control module to start publishing before going to external_control mode...")

                # Update objective position and publish it               
                status_check_rate.sleep()

        except KeyboardInterrupt :
            rospy.logerr('Keyboard Interrupt detected! Trying to land')

        # Stop objective update timer and land
        self.objective_timer.shutdown()
        self.dist_to_target_timer.shutdown()
        rospy.loginfo("Mission Completed. Landing...")
        rospy.wait_for_service('/mavros/cmd/land')


if __name__ == '__main__':
    '''
        Run the node
    '''
    try:

        # Create new control object
        control = LandingControl()

        # Run the mission control
        control.run()

    except rospy.ROSInterruptException:

        control.objective_timer.shutdown()
        control.dist_to_target_timer.shutdown()
        rospy.loginfo("Mission abruptly stopped. Trying to land...")
        rospy.wait_for_service('/mavros/cmd/land')