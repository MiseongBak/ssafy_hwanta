#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage



class GPS_to_UTM:
    def __init__(self, output_data_type):
        self.output_data_type = output_data_type
        rospy.init_node('GPS_to_UTM', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.utm_pub = rospy.Publisher("/utm", Float32MultiArray, queue_size=1)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)

        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_gps_data == True:
                self.utm_pub.publish(self.utm_msg)
                print("data published at '/utm' topic !")
            else:
                print("waiting for data...")
            rate.sleep()


    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        longitude = gps_msg.longitude
        latitude = gps_msg.latitude
        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]

        if self.output_data_type == "utm_xy":    
            self.utm_msg.data = [utm_x, utm_y]

        elif self.output_data_type == "simulator_map_xy":
            map_x = utm_x - gps_msg.eastOffset
            map_y = utm_y - gps_msg.northOffset
            self.utm_msg.data = [map_x, map_y]



if __name__ == '__main__':
    try:
        # GPS_to_UTM(output_data_type = 'utm_xy')
        GPS_to_UTM(output_data_type = 'simulator_map_xy')

    except rospy.ROSInterruptException:
        pass