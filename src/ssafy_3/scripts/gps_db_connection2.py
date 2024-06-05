#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pymysql
from morai_msgs.msg import GPSMessage
from std_msgs.msg import String
import time
import math

class GPSUpload:
    def __init__(self):
        # DB 연결 설정
        self.connection = pymysql.connect(host="stg-yswa-kr-practice-db-master.mariadb.database.azure.com",
                                          user="S10P23C110@stg-yswa-kr-practice-db-master.mariadb.database.azure.com",
                                          password="agmxq5a2TD",
                                          database="S10P23C110",
                                          cursorclass=pymysql.cursors.DictCursor)
        self.cursor = self.connection.cursor()
        self.last_upload_time = 0

        # ROS Subscriber와 Publisher 설정
        rospy.Subscriber("/gps", GPSMessage, self.callback)
        self.distance_publisher = rospy.Publisher("/cp_distance", String, queue_size=10)

        # 환자 GPS 데이터 초기 가져오기
        self.patient_lat = None
        self.patient_lon = None
        self.dest_lat = None
        self.dest_lon = None
        self.get_patient_gps_data()
        self.get_dest_gps_data()

    def callback(self, gps_msg):
        current_time = time.time()
        if current_time - self.last_upload_time >= 1.0:
            car_lat = gps_msg.latitude
            car_lon = gps_msg.longitude
            self.upload_car_gps_data(car_lat, car_lon)

            if self.patient_lat is not None and self.patient_lon is not None:
                distance1, distance2 = self.calculate_distance(car_lat, car_lon, float(self.patient_lat), float(self.patient_lon), float(self.dest_lat), float(self.dest_lon))
                if distance1 <= 50:
                    print("Patient is within 50 meters!")
                    self.distance_publisher.publish("near")
                elif distance2 <= 30:
                    print("Destination is within 50 meters!")
                    self.distance_publisher.publish("stop")
                else:
                    print("Patient is more than 50 meters away.")
                    self.distance_publisher.publish("far")

            self.last_upload_time = current_time

    def upload_car_gps_data(self, lat, lon):
        query = "UPDATE cars SET lat = %s, lon = %s WHERE id = 2"
        self.cursor.execute(query, (str(lat), str(lon)))
        self.connection.commit()
        print("GPS data uploaded to MariaDB")

    def get_patient_gps_data(self):
        query = "SELECT * FROM patients WHERE id = 2"
        self.cursor.execute(query)
        result = self.cursor.fetchone()
        if result:
            print("Patient GPS Data:", result)
            self.patient_lat = result['lat']
            self.patient_lon = result['lon']
        else:
            print("No patient data found.")
    
    def get_dest_gps_data(self):
        query = "SELECT * FROM destinations WHERE id = 2"
        self.cursor.execute(query)
        result = self.cursor.fetchone() 
        if result:
            print("Destination GPS Data:", result)
            self.dest_lat = result['lat']
            self.dest_lon = result['lon']
        else:
            print("No Destination data found.")

    def calculate_distance(self, lat1, lon1, lat2, lon2, lat3, lon3):
        # 지구 반지름 (미터 단위)
        R = 6371000
    
        # 위도, 경도를 라디안으로 변환
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi1 = math.radians(lat2 - lat1)
        delta_lambda1 = math.radians(lon2 - lon1)
        delta_phi2 = math.radians(lat3 - lat1)
        delta_lambda2 = math.radians(lon3 - lon1)
    
        # 허브사인 공식 사용
        a1 = math.sin(delta_phi1/2) * math.sin(delta_phi1/2) + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda1/2) * math.sin(delta_lambda1/2)
        c1 = 2 * math.atan2(math.sqrt(a1), math.sqrt(1-a1))

        a2 = math.sin(delta_phi2/2) * math.sin(delta_phi2/2) + \
            math.cos(phi2) * math.cos(phi2) * \
            math.sin(delta_lambda2/2) * math.sin(delta_lambda2/2)
        c2 = 2 * math.atan2(math.sqrt(a2), math.sqrt(1-a2))
    
        distance1 = R * c1
        distance2 = R * c2

        return distance1, distance2
            
if __name__ == '__main__':
    rospy.init_node('gps_parser', anonymous=True)
    gps = GPSUpload()
    
    rospy.spin()
