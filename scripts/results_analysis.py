#!/usr/bin/env python

import os
import csv
import sys
import numpy as np
import rospy
import message_filters
from geometry_msgs.msg  import PoseStamped
from sensor_msgs.msg    import Range
from std_msgs.msg       import String

#/uav_small_tag_xy /uav_big_tag_xy /uav/offboard/command_waypoint /dwm1001/tag/dronie/position /uav/mavros/setpoint_position/local 
#/uav/landing/distance /uav/landing/completed /dwm1001/tag/ugv/position /uav/mavros/local_position/pose /uav_tag_center_xy /uav_tag_dist


class Resuts_Analysis(): 
    def __init__(self):
        rospy.init_node('results_analysis', anonymous=False)

        self.cnt = 0

        self.file_fix = "ex_01_"

        self.file_name = "ex_01_"

        self.last_time = rospy.Time.now()

        self.traj_flag = False

        self.start_flag = False

        self.writing_now = False

        self.writing_stable = True

        self.end_flag = False

        self.complete_2s_flag = False

        self.uav_center = PoseStamped()

        self.uav_dist = Range()

        self.uav_waypoint = PoseStamped()

        self.uav_uwb_pos = PoseStamped()

        self.uav_vio_pos = PoseStamped()

        self.ros_out = String()

        self.uav_center_sub = rospy.Subscriber("/uav_tag_center_xy", PoseStamped, self.uav_center_cb)

        self.uav_dist_sub = rospy.Subscriber("/uav/landing/distance", Range, self. uav_landing_dist_cb)
        
        self.uav_waypoint_sub = rospy.Subscriber("/uav/offboard/command_waypoint", PoseStamped, self.uav_cmd_waypoint_cb)

        self.uav_uwb_position_sub = rospy.Subscriber("/dwm1001/tag/dronie/position", PoseStamped, self.uav_uwb_position_cb)

        self.uav_landing_complete_sub = rospy.Subscriber("/uav/landing/completed", String, self.uav_landing_complete_2s_cb)

        self.uav_vio_sub = rospy.Subscriber("/uav/mavros/vision_pose/pose", PoseStamped, self.uav_vio_cb)

        self.uav_ros_out = rospy.Subscriber("/rosout", String, self.ros_out_cb)

    def ros_out_cb(self, msg):
        

    def uav_vio_cb(self, pos):
        self.uav_vio_pos = pos

    def uav_center_cb(self, pos):
        self.uav_center = pos

    
    def uav_uwb_position_cb(self, pos):
        self.uav_uwb_pos = pos


        # 2s after landing completion
        if self.complete_2s_flag:
            if (self.uav_uwb_pos.header.stamp - self.last_time) > rospy.Duration(3.0):
                self.complete_2s_flag = False
                rospy.loginfo(">>>>>>> Landing 3s: %d Completed<<<<<<<<", self.cnt)
                self.cnt = self.cnt + 1
                
            with open(self.landing_name, mode='a') as f:
                writer = csv.writer(f) 
                row = (self.uav_uwb_pos.header.stamp, self.uav_center.pose.position.x, self.uav_center.pose.position.y,
                       self.uav_dist.range)
                writer.writerow(row)

        # Trajectories 
            
        if self.traj_flag:
            with open(self.traj_name, mode='a') as f:
                writer = csv.writer(f) 
                row = (self.uav_uwb_pos.header.stamp, self.uav_waypoint.pose.position.x, self.uav_waypoint.pose.position.y,
                       self.uav_vio_pos.pose.position.x, self.uav_vio_pos.pose.position.y)
                writer.writerow(row)

        # if self.writing_now:
        #     with open(self.file_name, mode='a') as f:
        #         writer = csv.writer(f) 
        #         row = (self.uav_waypoint.header.stamp, self.uav_center.pose.position.x, self.uav_center.pose.position.y,
        #                self.uav_dist.range, self.uav_waypoint.pose.position.x, self.uav_waypoint.pose.position.y,)
        #         writer.writerow(row)
        # else:
        #     if self.start_flag:
        #         print(self.cnt)
        #         self.file_name = self.file_fix+ str(self.cnt) + ".csv"
        #         print("Begin {} Writing!".format(self.file_name))
        #         with open(self.file_name, mode='wt') as f:
        #             writer = csv.writer(f) 
        #             row = ('timestamp', 'uav_center_x', 'uav_center_y', 'uav_dist', 'uav_waypoint_x', 'uav_waypoint_y',)
        #             writer.writerow(row)
        #         self.start_flag = False


    def uav_landing_dist_cb(self, dist):
        self.uav_dist = dist


    def uav_cmd_waypoint_cb(self, pos):
        self.uav_waypoint = pos
        if self.uav_waypoint.pose.position.x > 0.4 and not self.traj_flag:
            self.traj_flag =  True
            self.traj_name = self.file_fix + str(self.cnt) + "_traj_" + ".csv"
            rospy.loginfo(">>>>>>> Trajectory Writing File: %s <<<<<<<<", self.landing_name)
            with open(self.landing_name, mode='wt') as f:
                writer = csv.writer(f)
                row = ('timestamp', 'uav_vio_x', 'uav_vio_y', 'uav_waypoint_x', 'uav_waypoint_y')
                writer.writerow(row)
        elif self.uav_waypoint.pose.position.x < 0.4 and self.traj_flag:
            rospy.loginfo(">>>>>>> Trajectory %d Completed <<<<<<<<", self.cnt)
            self.traj_flag = False 

            # if not self.writing_now:
            #     if self.end_flag: self.end_flag = False
            #     else:
            #         self.start_flag = True
            #         self.writing_now = True
            #         self.writing_stable = True


    def uav_landing_complete_cb(self, msg):
        if  "Landing" in msg.data:
            self.end_flag = True
            self.writing_now = False
            self.start_flag = True
            if self.writing_stable:
                self.writing_stable = False
                self.landing_name = self.file_fix + str(self.cnt) + "_landing_" + ".csv"
                rospy.loginfo(">>>>>>> Landing 3s Writing File: %s Completed <<<<<<<<", self.landing_name)
                with open(self.landing_name, mode='wt') as f:
                    writer = csv.writer(f)
                    row = ('timestamp', 'uav_center_x', 'uav_center_y', 'uav_dist')
                    writer.writerow(row)
                self.cnt = self.cnt + 1
            with open(self.landing_name, mode='a') as f:
                writer = csv.writer(f) 
                row = (self.uav_waypoint.header.stamp, self.uav_center.pose.position.x, self.uav_center.pose.position.y,
                       self.uav_dist.range)
                writer.writerow(row)

    def uav_landing_complete_2s_cb(self, msg):
        if "Landing" in msg.data and not self.complete_2s_flag:
            self.complete_2s_flag = True
            self.last_time = self.uav_uwb_pos.header.stamp
            self.landing_name = self.file_fix + str(self.cnt) + "_landing_" + ".csv"
            rospy.loginfo(">>>>>>> Landing 3s Writing File: %s <<<<<<<<", self.landing_name)
            with open(self.landing_name, mode='wt') as f:
                writer = csv.writer(f)
                row = ('timestamp', 'uav_center_x', 'uav_center_y', 'uav_dist')
                writer.writerow(row)

            


    # def run(self, event):
    #     if self.start_flag:
    #         self.start_flag = False
    #         print(self.cnt)
    #         self.file_name = self.file_fix + str(self.cnt) + ".csv"
    #         print("Begin {} Writing!".format(self.file_name))
    #         with open(self.file_name, mode='wt') as f:
    #             writer = csv.writer(f)
    #             row = ('timestamp', 'uav_center_x', 'uav_center_y', 'uav_dist', 'uav_waypoint_x', 'uav_waypoint_y',)
    #             writer.writerow(row)
    
    #     if self.writing_now and not self.start_flag:
    #         with open(self.file_name, mode='a') as f:
    #             writer = csv.writer(f) 
    #             row = (self.uav_waypoint.header.stamp, self.uav_center.pose.position.x, self.uav_center.pose.position.y,
    #                    self.uav_dist.range, self.uav_waypoint.pose.position.x, self.uav_waypoint.pose.position.y,)
    #             writer.writerow(row)

        # if self.writing :

        #     with open(self.base_file_name + "{}".format(self.counter), "a") as f :
        #         f.write("{}, {}, {}, {}")

        # if self.writing_stable :
            
        #     with open(self.base_file_name + "{}_stable".format(self.counter), "a") as f :
        #         f.write()

    


# class results_analysis() :
#     def __init__(self):
#         # Init node
#         rospy.init_node('results_analysis', anonymous=False)

#         self.big_tag_pos = PoseStamped()

#         self.small_tag_pos = PoseStamped()

#         self.center_tag_pos = PoseStamped()

#         self.tag_range = Range()

#         self.tag_dist = 0.0

#         self.big_tag_sub = rospy.Subscriber("uav_big_tag_xy", PoseStamped, self.big_tag_cb)
#         self.small_tag_sub = rospy.Subscriber("uav_small_tag_xy", PoseStamped, self.small_tag_cb)

#         self.center_pub = rospy.Publisher("/uav_center_xy", PoseStamped, queue_size=10)
#         self.dist_pub = rospy.Publisher("/uav_tag_range", Range, queue_size=10)

#         self.timer = rospy.Timer(rospy.Duration(0.1), self.run)

#     def big_tag_cb(self, pos):
#         self.big_tag_pos = pos
#         self.big_tag_pos.header.stamp = rospy.Time.now()

#     def small_tag_cb(self, pos):
#         self.small_tag_pos = pos
#         self.small_tag_pos.header.stamp = rospy.Time.now()

#     def tag_dist_cal(self):
#         if rospy.Time.now() - self.big_tag_pos.header.stamp > rospy.Duration(0.2) :
#             print("Big tag laggin behind")
#             return
#         if rospy.Time.now() - self.small_tag_pos.header.stamp > rospy.Duration(0.2) :
#             print("Small tag laggin behind")
#             return
#         self.tag_dist = np.sqrt(np.power(self.big_tag_pos.pose.position.x - self.small_tag_pos.pose.position.x, 2) \
#                         + np.power(self.big_tag_pos.pose.position.y - self.small_tag_pos.pose.position.y, 2))
#         self.tag_range.header.stamp = rospy.Time.now()
#         self.tag_range.range = self.tag_dist
#         self.dist_pub.publish(self.tag_range)

#     def drone_center_cal(self):
#         if rospy.Time.now() - self.big_tag_pos.header.stamp > rospy.Duration(0.2) :
#             # print("Big tag laggin behind")
#             return
#         if rospy.Time.now() - self.small_tag_pos.header.stamp > rospy.Duration(0.2) :
#             # print("Small tag laggin behind")
#             return
#         self.center_tag_pos.pose.position.x = (self.big_tag_pos.pose.position.x + self.small_tag_pos.pose.position.x)/2.0
#         self.center_tag_pos.pose.position.y = (self.big_tag_pos.pose.position.y + self.small_tag_pos.pose.position.y)/2.0
#         self.center_tag_pos.header.stamp = rospy.Time.now()
#         self.center_pub.publish(self.center_tag_pos)
        
#     def run(self, event):
#          self.tag_dist_cal()
#          self.drone_center_cal()

if __name__ == '__main__':
    '''
        Run the node
    '''
    # Create new control object
    # dpk = dronetag_position_check()

    # Run the mission control
    # dpk.run()
    #dpk.tag_dist_cal()
    #dpk.drone_center_cal()

    r_a = Resuts_Analysis()

    # rospy.Timer(rospy.Duration(0.1), r_a.run)

    rospy.spin()
