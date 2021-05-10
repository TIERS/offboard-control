#!/usr/bin/env python

import os
import sys
import time
import rospy
import serial
import random

from geometry_msgs.msg  import Pose

from std_msgs.msg       import Float64
from std_msgs.msg       import String


class dwm1001_localizer:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """

        # Init node
        rospy.init_node('DWM1001_Active_{}'.format(random.randint(0,100000)), anonymous=False)

        # Get port and tag name
        self.dwm_port = rospy.get_param('~port')
        self.tag_name = rospy.get_param('~tag_name')
        self.network = rospy.get_param('~network', "default")
        self.verbose = rospy.get_param('~verbose', False)
        
        # Set a ROS rate
        self.rate = rospy.Rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}

        self.measure_matrix = []
        self.new_measure_received = False
        
        # Serial port settings
        self.serialPortDWM1001 = serial.Serial(
            port = self.dwm_port,
            baudrate = 115200,
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
        self.serialPortDWM1001.close()
        # sleep for one sec
        time.sleep(1)
        # open serial port
        self.serialPortDWM1001.open()

        # check if the serial port is opened
        if(self.serialPortDWM1001.isOpen()):
            rospy.loginfo("Port opened: "+ str(self.serialPortDWM1001.name) )
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001.name))

        self.pub = rospy.Publisher("/uwb/matrix/distances", String, queue_size=100)

        try:

            while not rospy.is_shutdown():
                # just read everything from serial port
                serialReadLine = self.serialPortDWM1001.read_until()
                # print(serialReadLine)
                # try:
                self.publishTagPositions(serialReadLine)

                # except IndexError:
                #     rospy.loginfo("Found index error in the network array! DO SOMETHING!")

        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board")


    def to_distance(self, mean_lsb, mean_msb):
        lsb = float(mean_lsb)
        msb = float(mean_msb)

        return (msb * 256 + lsb) / 100

    def publishTagPositions(self, serialData):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """
  
        arrayData = [x  for x in serialData.strip().split(' ')] 
        # print(serialData)

        if len(self.measure_matrix) < 4 and self.new_measure_received:
            self.measure_matrix.append([x  for x in serialData.strip().split(',')])
        elif len(self.measure_matrix) is 4:
            
            str12 =  " 1 -> 2 : " + str( self.to_distance( self.measure_matrix[0][5] , self.measure_matrix[0][4]))
            str13 =  " 1 -> 3 : " + str( self.to_distance( self.measure_matrix[0][9] , self.measure_matrix[0][8]))
            str14 =  " 1 -> 4 : " + str( self.to_distance( self.measure_matrix[0][13], self.measure_matrix[0][12]))
            str21 =  " 2 -> 1 : " + str( self.to_distance( self.measure_matrix[1][1] ,self.measure_matrix[1][0])  )
            str23 =  " 2 -> 3 : " + str( self.to_distance( self.measure_matrix[1][9] ,self.measure_matrix[1][8])  )
            str24 =  " 2 -> 4 : " + str( self.to_distance( self.measure_matrix[1][13],self.measure_matrix[1][12])  )
            str31 =  " 3 -> 1 : " + str( self.to_distance( self.measure_matrix[2][1] ,self.measure_matrix[2][0])  )
            str32 =  " 3 -> 2 : " + str( self.to_distance( self.measure_matrix[2][5] ,self.measure_matrix[2][4])  )
            str34 =  " 3 -> 4 : " + str( self.to_distance( self.measure_matrix[2][13] ,self.measure_matrix[2][12])  )
            str41 =  " 4 -> 1 : " + str( self.to_distance( self.measure_matrix[3][1] ,self.measure_matrix[3][0] )  )
            str42 =  " 4 -> 2 : " + str( self.to_distance( self.measure_matrix[3][5] ,self.measure_matrix[3][4] )  )
            str43 =  " 4 -> 3 : " + str( self.to_distance( self.measure_matrix[3][9] ,self.measure_matrix[3][8] )  )

            self.pub.publish(str12)
            self.pub.publish(str13)
            self.pub.publish(str14)
            self.pub.publish(str21)
            self.pub.publish(str23)
            self.pub.publish(str24)
            self.pub.publish(str31)
            self.pub.publish(str32)
            self.pub.publish(str34)
            self.pub.publish(str41)
            self.pub.publish(str42)
            self.pub.publish(str43)
            # print("Message published !!! ")

            # print(self.measure_matrix)
            self.measure_matrix = []
            self.new_measure_received = False

        # print(arrayData)
        if 'responder' in arrayData :
            # print("New data received ...") 
            self.new_measure_received = True


if __name__ == '__main__':
    try:
        dwm1001 = dwm1001_localizer()
        dwm1001.main()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
