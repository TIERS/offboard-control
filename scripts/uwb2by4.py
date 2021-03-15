#!/usr/bin/env python
""" 
    For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""

import rospy, time, serial, os, sys, random

from geometry_msgs.msg  import Pose
from std_msgs.msg       import Float64

# initialize ros rate 10hz


# initialize serial port connections



class dwm1001_localizer:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """

        # Init node
        rospy.init_node('DWM1001_Active_{}'.format(random.randint(0,100000)), anonymous=False)

        # Get port and tag name
        self.dwm_port1 = rospy.get_param('~port1', '/dev/ttyACM0')
        self.dwm_port2 = rospy.get_param('~port2', '/dev/ttyACM1')
        self.tag_name = rospy.get_param('~tag_name', "1by4")
        self.network = rospy.get_param('~network', "default")
        self.verbose = rospy.get_param('~verbose', False)
        
        # Set a ROS rate
        self.rate = rospy.Rate(100)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        
        # Serial port settings
        self.serialPortDWM1001_1 = serial.Serial(
            port = self.dwm_port1,
            baudrate = 115200,
            timeout = 0.1,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )
        
        self.serialPortDWM1001_2 = serial.Serial(
            port = self.dwm_port2,
            baudrate = 115200,
            timeout = 0.1,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )

    def main(self) :
        """
        Initialize port and dwm1001 api
        :param:
        :returns: none
        """

        # close the serial port in case the previous run didn't closed it properly
        self.serialPortDWM1001_1.close()
        self.serialPortDWM1001_2.close()
        # sleep for one sec
        time.sleep(1)
        # open serial port
        self.serialPortDWM1001_1.open()
        self.serialPortDWM1001_2.open()

        # check if the serial port is opened
        if(self.serialPortDWM1001_1.isOpen()):
            rospy.loginfo("Port opened: "+ str(self.serialPortDWM1001_1.name) )
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001_1.name))
        
        if(self.serialPortDWM1001_2.isOpen()):
            rospy.loginfo("Port opened: "+ str(self.serialPortDWM1001_2.name) )
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001_2.name))

        try: 
            tags = ['A', 'B', 'C', 'D']
            # Counting the msgs received and processed to check the system status
            msg_portO_cnt = [0,0,0,0]
            msg_portP_cnt = [0,0,0,0]
            while not rospy.is_shutdown():
                self.serialPortDWM1001_1.write("S".encode())
                # just read everything from serial port 
                for _ in range(4): 
                    serialReadLine = self.serialPortDWM1001_1.read_until()
                    if not serialReadLine:
                        rospy.logwarn("Port O: Read a empty line from Serial Port")
                        continue
                    self.publishTagPositions('O',serialReadLine)
                    msg_portO_cnt[tags.index(serialReadLine[9])] += 1
                    self.rate.sleep()

                self.serialPortDWM1001_2.write("S".encode()) 
                for _ in range(4): 
                    serialReadLine = self.serialPortDWM1001_2.read_until()
                    if not serialReadLine:
                        rospy.logwarn("Port P: Read a empty line from Serial Port")
                        continue
                    self.publishTagPositions('P',serialReadLine)  
                    msg_portP_cnt[tags.index(serialReadLine[9])] += 1 
                    self.rate.sleep()

                if sum(msg_portO_cnt) > 100:
                    rospy.loginfo("Running well. Done O-> {}, P-> {} times !".format(msg_portO_cnt, msg_portP_cnt))
                    msg_portO_cnt = [0,0,0,0]
                    msg_portP_cnt = [0,0,0,0]
                # except IndexError:
                #     rospy.loginfo("Found index error in the network array! DO SOMETHING!")

        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board")


    def publishTagPositions(self, initTagID, serialData):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """
  
        arrayData = [x  for x in serialData.strip().split(' ')] 
        if self.verbose :
            print(" Input : ", arrayData, " | Initial ID : ", initTagID,  " | length: ", len(arrayData))

        if(len(arrayData) is not 4):
            rospy.logwarn("Wrong length!")
            return
 
        node_id = initTagID + '2' + arrayData[1]
 
        if node_id not in self.topics :
            print("NodeID: ", node_id)
            # tag_name = ""
            # if node_id == 'A':
            #     tag_name = "tag_A"
            # elif node_id == 'B':
            #     tag_name = "tag_B"
            # elif node_id == 'C':
            #     tag_name = "tag_C"
            # elif node_id == 'D':
            #     tag_name = "tag_D"
                
            # else:
            #     rospy.logwarn("WRONG NODE ID; Return !")
            #     return 
  
            self.topics[node_id] = rospy.Publisher('/dwm1001/'+ node_id + "/distance", Float64, queue_size=100)

        try :
            dist = float(arrayData[3])
            if self.verbose :
                print("Publish distance topic : ", dist)
            self.topics[node_id].publish(dist)
        except :
            rospy.logwarn("WRONG DISTANCE data!")

if __name__ == '__main__':
    try:
        dwm1001 = dwm1001_localizer()
        dwm1001.main()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
