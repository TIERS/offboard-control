#!/usr/bin/env python
# license removed for brevity
import sys
import time
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

def offboard_test():

    print("Starting...")
    rospy.init_node('test_offboard', anonymous=False)

    pub_mode = rospy.Publisher('/uav/offboard/mode', String, queue_size=10)
    pub_pose = rospy.Publisher('/uav/offboard/command_waypoint', PoseStamped, queue_size=10)

    mode = String()
    mode.data = "external_control"
    print("Publishing...")
    # for i in range(2) :
    #     print(i)
    #     time.sleep(0.05)
    #     pub_mode.publish(mode)
    
    rate = rospy.Rate(10) # 10hz
    cnt = 0
    while not rospy.is_shutdown():

        goalMsg = PoseStamped()
        goalMsg.header.frame_id = "test"
        goalMsg.pose.orientation.z = 0.0
        goalMsg.pose.orientation.w = 1.0

        goalMsg.header.stamp = rospy.Time.now()
        goalMsg.pose.position.x = 1.2
        goalMsg.pose.position.y = 0
        goalMsg.pose.position.z = 1

        cnt += 1
        if cnt > 100 :
            sys.stdout.write("\rTesting VOI safety fence now... ")
            sys.stdout.write(str(cnt))
            sys.stdout.flush()
            goalMsg.pose.position.x = 1.8

        pub_pose.publish(goalMsg)
        if cnt < 10 :
            pub_mode.publish(mode)
        rate.sleep()

if __name__ == '__main__':
    try:
        offboard_test()
    except rospy.ROSInterruptException:
        pass
