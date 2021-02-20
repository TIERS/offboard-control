#!/usr/bin/env python
# license removed for brevity
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

def offboard_test():

    rospy.init_node('test_offboard', anonymous=False)

    pub_mode = rospy.Publisher('/offboard/mode', String, queue_size=10)
    pub_pose = rospy.Publisher('/uav/external_waypoints', PoseStamped, queue_size=10)

    mode = String()
    mode.data = "external_control"
    pub_mode.publish(mode)
    
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
        
        pub_pose.publish(goalMsg)

        cnt += 1
        if cnt > 100 :
            sys.stdout.write("\rTesting VOI safety fence now...")
            for i in range(100,cnt) :
                sys.stdout.write('.')
            sys.stdout.flush()
            goalMsg.pose.position.x = 1.8

        rate.sleep()

if __name__ == '__main__':
    try:
        offboard_test()
    except rospy.ROSInterruptException:
        pass
