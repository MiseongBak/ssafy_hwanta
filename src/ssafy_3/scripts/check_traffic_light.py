#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from morai_msgs.msg import GetTrafficLightStatus
from std_msgs.msg import String, Int32


class LocalPathPub:
    def __init__(self):
        self.traffic_light_pub =  rospy.Publisher('/check_traffic_light',String, queue_size=1 )
        self.traffic_light_num = rospy.Publisher('/check_traffic_light_num', Int32, queue_size=1 )
        self.msg = String()
        self.num = -1
        self.nowIndex = String()
        self.traffic_light_list = [
            ["C119BS010024", 75.6222687308,    1250.58971986,    0.0],
            ["C119BS010025", 136.604752,    1351.215942,    -0.511265],
            ["C119BS010028", 140.65213, 1458.177124,    -0.511265],
            ["C119BS010033", 139.172119,    1596.130249,    -0.511265],
            ]
        
        
        rospy.init_node('traffic_listener', anonymous=True)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_light_callback)
        rospy.spin()

    def traffic_light_callback(self, data):
        #rospy.loginfo('-------------------- Traffic Light Vehicle -------------------------')
        #rospy.loginfo("Traffic Light Idx    : {}".format(data.trafficLightIndex))
        # rospy.loginfo("Traffic Light Status : {}".format(data.trafficLightStatus))
        # rospy.loginfo("Traffic Light Type   : {}".format(data.trafficLightType))
        
        self.nowIndex = data.trafficLightIndex
        print(self.nowIndex)
        for i in range(4):
            if i == 0:
                if self.traffic_light_list[i][0] == self.nowIndex:
                    if data.trafficLightStatus == 1 or data.trafficLightStatus == 4 or data.trafficLightStatus == 16:
                        self.msg = "red_light"
                        self.num = i
                    else:
                        self.msg = "green_light"
            else :
                if self.traffic_light_list[i][0] == self.nowIndex:
                    if data.trafficLightStatus == 1 or data.trafficLightStatus == 4:
                        self.msg = "red_light"
                        self.num = i
                    else:
                        self.msg = "green_light"

        # rospy.loginfo(self.msg)
        # print(type(self.msg))
        self.traffic_light_pub.publish(self.msg)
        self.traffic_light_num.publish(self.num)

if __name__ == '__main__':
    try:
        LocalPathPub()
    except rospy.ROSInterruptException:
        pass