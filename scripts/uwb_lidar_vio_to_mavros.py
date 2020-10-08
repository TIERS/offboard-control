#!/usr/bin/env python

import sys
import rospy

from math                import atan2

from sensor_msgs.msg     import Range

from geometry_msgs.msg     import Pose
from geometry_msgs.msg     import PoseStamped

from nav_msgs.msg       import Odometry

from tf.transformations import quaternion_from_euler

class UWB_Lidar_Position :

    def __init__(self) :
        '''
            Init node, subscribe to position data
        ''' 
        # Init node
        rospy.init_node("UWB_and_Lidar_Positioning", anonymous=False)

        
        self.lidarz = 0
        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.orientation_w = 1

        # Publish data
        self.local_pos_pub      = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        # Subscribe to pose topics
        self.uwb_subscriber = rospy.Subscriber("/dwm1001/tag/drone/position", Pose, self.uwb_cb)
        self.lidar_subscriber = rospy.Subscriber("/tfmini_ros_node/TFmini", Range, self.lidar_cb)

        

    def run(self) :
        '''
            Infinite loop with 20 Hz sleep
        ''' 
        # Set Rospy rate
        rate = rospy.Rate(20.0)

        # Infinite loop
        try:
            while not rospy.is_shutdown() :
                rate.sleep()

        except KeyboardInterrupt :
             rospy.logerr('Keyboard Interrupt detected!')

    def uwb_cb(self, pos) :
        '''
            Get position from UWB, Lidar and VIO (T265)
        '''
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = pos.position.x
        pose.pose.position.y = pos.position.y
        pose.pose.position.z = self.lidarz

        pose.pose.orientation.x = self.orientation_x
        pose.pose.orientation.y = self.orientation_y
        pose.pose.orientation.z = self.orientation_z
        pose.pose.orientation.w = self.orientation_w

        self.local_pos_pub.publish(pose)


    def lidar_cb(self, pos) :
        '''
            Update ground truth position
        '''
        self.lidarz = pos.range
        
    def vio_cb(self, odom) :
        '''
            Update orientation from VIO
        '''
        self.orientation_x = odom.pose.pose.orientation.x
        self.orientation_y = odom.pose.pose.orientation.y
        self.orientation_z = odom.pose.pose.orientation.z
        self.orientation_w = odom.pose.pose.orientation.w


if __name__ == '__main__':
    '''
        Run the node
    '''
    try:

        # Create new object
        testing = UWB_Lidar_Position()

        # Loop while reading data
        testing.run()

    except rospy.ROSInterruptException:

        pass

