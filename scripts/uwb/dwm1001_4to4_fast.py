#!/usr/bin/env python
""" For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""

__author__     = "Jorge Pena Queralta; Qingqing Li  "
__version__    = "0.1"
__maintainer__ = "Jorge Pena Queralta"
__email__      = "jopequ@utu.fi, qingqli@utu.fi"
__status__     = "Development"


import rospy, time, serial, os, sys, random

from geometry_msgs.msg  import Pose
from std_msgs.msg       import Float64
from std_msgs.msg       import String
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
        self.dwm_port = rospy.get_param('~port', '/dev/uwb')
        self.tag_name = rospy.get_param('~tag_name', 'my_node')
        self.network = rospy.get_param('~network', "default")
        self.verbose = rospy.get_param('~verbose', False)
        
        # Set a ROS rate
        self.rate = rospy.Rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        self.msg_counter = 0

        self.error_buffer    =   ['1 -> 2', '1 -> 3', '1 -> 4']
        self.error_cnt       =   [0,0,0]

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

        # Publishers
        transceiver_names = ["A", "B", "C", "D"]
        self.publishers = [[None for _ in range(4)] for _ in range(4)]
        for idx, tr_from in enumerate(transceiver_names) :
            for jdx, tr_to in enumerate(transceiver_names) :
                if jdx != idx :
                    self.publishers[idx][jdx] = rospy.Publisher("/uwb/distance/from/{}/to/{}".format(tr_from, tr_to), Float64, queue_size=5)
    

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

        try:

            while not rospy.is_shutdown():
                # just read everything from serial port
                serialReadLine = self.serialPortDWM1001.read_until()
                # print(serialReadLine)
                # print("===>>>")
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

    def reset_error(self):
        self.error_cnt[0] = 0
        self.error_cnt[1] = 0
        self.error_cnt[2] = 0
        # print("Rest error: ", self.error_cnt)



    def publishTagPositions(self, serialData):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """
        
        # Update the wrong message counts
        for idx, er in enumerate(self.error_buffer): 
            if er in serialData:
                self.error_cnt[idx] += 1 

        # Output error, if request 10 times failure 
        for idx, cnt in enumerate(self.error_cnt):
            if cnt > 10:
                # print self.error_buffer[idx]
                print( "===>>> ERROR: Tag {} is Lost".format( self.error_buffer[idx][-1] ) )
                # print(self.error_cnt )
                self.reset_error()    

        arrayData = [x  for x in serialData.strip().split(' ')]
 
        # Initial data checking
        if len(self.measure_matrix) < 4 and self.new_measure_received:
            # print([x  for x in serialData.strip().split(',')], '        ', len([x  for x in serialData.strip().split(',')]) )
            data_length = len([x  for x in serialData.strip().split(',')])
            if(data_length is 17):
                self.measure_matrix.append([x  for x in serialData.strip().split(',')])
            else:
                self.new_measure_received = False
                self.measure_matrix = []
                print("Initial Wrong , reinitial !")
        # Publish the measurements
        elif len(self.measure_matrix) is 4:
            pub = rospy.Publisher("/tags/distance", String, queue_size=100)
            now = rospy.get_rostime()
            str12 =  "{} {} : 1 -> 2 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[0][5] , self.measure_matrix[0][4]))
            str13 =  "{} {} : 1 -> 3 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[0][9] , self.measure_matrix[0][8]))
            str14 =  "{} {} : 1 -> 4 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[0][13], self.measure_matrix[0][12]))
            str21 =  "{} {} : 2 -> 1 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[1][1] ,self.measure_matrix[1][0])  )
            str23 =  "{} {} : 2 -> 3 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[1][9] ,self.measure_matrix[1][8])  )
            str24 =  "{} {} : 2 -> 4 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[1][13],self.measure_matrix[1][12])  )
            str31 =  "{} {} : 3 -> 1 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[2][1] ,self.measure_matrix[2][0])  )
            str32 =  "{} {} : 3 -> 2 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[2][5] ,self.measure_matrix[2][4])  )
            str34 =  "{} {} : 3 -> 4 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[2][13] ,self.measure_matrix[2][12])  )
            str41 =  "{} {} : 4 -> 1 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[3][1] ,self.measure_matrix[3][0] )  )
            str42 =  "{} {} : 4 -> 2 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[3][5] ,self.measure_matrix[3][4] )  )
            str43 =  "{} {} : 4 -> 3 : ".format(now.secs, now.nsecs) + str( self.to_distance( self.measure_matrix[3][9] ,self.measure_matrix[3][8] )  )

            self.reset_error()

            for i in range(4) :
                for j in range(4) :
                    if i != j :
                        self.publishers[i][j].publish(
                            Float64(
                                data=self.to_distance(
                                    self.measure_matrix[i][4*j + 1],
                                    self.measure_matrix[i][4*j]
                                )
                            )
                        )

            pub.publish(str12)
            pub.publish(str13)
            pub.publish(str14)
            pub.publish(str21)
            pub.publish(str23)
            pub.publish(str24)
            pub.publish(str31)
            pub.publish(str32)
            pub.publish(str34)
            pub.publish(str41)
            pub.publish(str42)
            pub.publish(str43)
            print("{} : message published ".format(self.msg_counter))
            self.msg_counter += 1

            # print(self.measure_matrix)
            self.measure_matrix = []
            self.new_measure_received = False
 
        if 'Sending' in  arrayData :
            # print("New data received ...") 
            self.new_measure_received = True
 

if __name__ == '__main__':
    try:
        dwm1001 = dwm1001_localizer()
        dwm1001.main()
    except rospy.ROSInterruptException:
        pass