#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from morai_msgs.msg import GPSMessage
from std_msgs.msg import String
from nav_msgs.msg import Odometry,Path

# local_path_pub 은 global Path (전역경로) 데이터를 받아 Local Path (지역경로) 를 만드는 예제입니다.
# Local Path (지역경로) 는 global Path(전역경로) 에서 차량과 가장 가까운 포인트를 시작으로 만들어 집니다.

# 노드 실행 순서 
# 1. Global Path 와 Odometry 데이터 subscriber 생성 
# 2. Local Path publisher 선언
# 3. Local Path 의 Size 결정
# 4. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
# 5. Global Path 에서 차량 위치와 가장 가까운 포인트(Currenty Waypoint) 탐색
# 6. 가장 가까운 포인트(Currenty Waypoint) 위치부터 Local Path 생성 및 예외 처리 
# 7. Local Path 메세지 Publish


class local_path_pub :
    def __init__(self):
        rospy.init_node('check_path_pub', anonymous=True)
        rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        rospy.Subscriber("/lattice_path", Path, self.lattice_path_callback)
        rospy.Subscriber('/lane_path', Path, self.lane_path_callback)

        self.message =  rospy.Publisher('/check',String, queue_size=1 )
        self.path_pub = rospy.Publisher('/selected_path',Path, queue_size=1)
        
        # 초기화
        self.is_lattice_path = False
        self.is_lane_path = False
        self.gps_valid = False
        self.lattice_path = Path()
        self.lane_path = Path()
        self.selected_path = Path()
        self.msg = String()

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.gps_valid:
                self.selected_path = self.lattice_path
                self.msg = "lattice"
            else:
                self.selected_path = self.lane_path
                self.msg = "lane"
            self.path_pub.publish(self.selected_path)
            self.message.publish(self.msg)

            rate.sleep()     

    def navsat_callback(self, gps_msg):
        if gps_msg.latitude and gps_msg.longitude:
            self.gps = gps_msg
            self.gps_valid = True
        else:
            self.gps_valid = False

    def lattice_path_callback(self, msg):
        self.is_lattice_path = True
        self.lattice_path = msg

    def lane_path_callback(self, msg):
        self.is_lane_path = True
        self.lane_path = msg

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass
