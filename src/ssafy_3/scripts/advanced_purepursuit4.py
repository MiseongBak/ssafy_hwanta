#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList
from std_msgs.msg import String
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        local_path_name = arg[1]
        rospy.Subscriber(local_path_name, Path, self.path_callback)

        # rospy.Subscriber("/lattice_path", Path, self.path_callback)

        rospy.Subscriber("/global_path2", Path, self.global_path_callback )
        rospy.Subscriber("/odom", Odometry, self.odom_callback )
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus , self.status_callback )
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)
        
        rospy.Subscriber("/check", String, self.message_callback)
        rospy.Subscriber("/cp_distance", String, self.distance_callback)

        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False 
        self.is_status = False
        self.is_global_path = False

        self.is_enter = False
        self.cnt = 0
        self.pause_time = 0

        self.is_look_forward_point = False

        self.check_msg = String()
        self.distance_msg = String()

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 2.6
        self.lfd = 8
        self.min_lfd = 6
        self.max_lfd = 30
        self.lfd_gain = 0.79
        self.target_velocity = 50

        self.pid = pidControl()
        self.adaptive_cruise_control = AdaptiveCruiseControl(velocity_gain = 0.5, distance_gain = 1, time_gap = 0.8, vehicle_length = 2.7)
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)
        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_path == True and self.is_odom == True and self.is_status == True:
                prev_time = time.time()

                # global_obj,local_obj
                result = self.calc_vaild_obj([self.current_postion.x,self.current_postion.y,self.vehicle_yaw],self.object_data)
                
                global_npc_info = result[0] 
                local_npc_info = result[1] 
                global_ped_info = result[2] 
                local_ped_info = result[3] 
                global_obs_info = result[4] 
                local_obs_info = result[5] 
                
                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6
                # if self.check_msg == "lane":
                #     self.target_velocity = 5

                if not self.distance_msg == String("far"):
                    if self.distance_msg == String("near") and self.pause_time < 450:
                        self.pause_time = self.pause_time + 1
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 1
                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                        continue
        
                    elif self.distance_msg == String("stop"):
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 1
                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                        continue

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0

                self.adaptive_cruise_control.check_object(self.path ,global_npc_info, local_npc_info
                                                                    ,global_ped_info, local_ped_info
                                                                    ,global_obs_info, local_obs_info)
                self.target_velocity = self.adaptive_cruise_control.get_target_velocity(local_npc_info, local_ped_info, local_obs_info,
                                                                                                        self.status_msg.velocity.x, self.target_velocity/3.6)

                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)
                
                if self.check_msg == String("lane") and output > 0:
                    output = output * 0.2
                
                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                if self.check_msg == String("lane") and self.cnt < 60:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 0.08
                    self.ctrl_cmd_msg.steering = 0
                    self.cnt = self.cnt + 1
                elif self.check_msg == String("lane") and output > 0:
                    self.ctrl_cmd_msg.accel *= 0.2

                if self.check_msg == String("lattice") and self.cnt:
                    self.cnt = 0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def message_callback(self,msg):
        self.check_msg = msg


    def distance_callback(self,msg):
        self.distance_msg = msg

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg): 
        self.is_status=True
        self.status_msg=msg    
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True

    def object_info_callback(self,data): 
        self.is_object_info = True
        self.object_data = data 
    
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint
    
    def calc_vaild_obj(self,status_msg,object_data):
        
        self.all_object = object_data        
        ego_pose_x = status_msg[0]
        ego_pose_y = status_msg[1]
        ego_heading = status_msg[2]
        
        global_npc_info = []
        local_npc_info  = []
        global_ped_info = []
        local_ped_info  = []
        global_obs_info = []
        local_obs_info  = []

        num_of_object = self.all_object.num_of_npcs + self.all_object.num_of_obstacle + self.all_object.num_of_pedestrian        
        if num_of_object > 0:

            #translation
            tmp_theta=ego_heading
            tmp_translation=[ego_pose_x, ego_pose_y]
            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                            [sin(tmp_theta),  cos(tmp_theta), tmp_translation[1]],
                            [0             ,               0,                  1]])
            tmp_det_t=np.array([[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0] * tmp_translation[0] + tmp_t[1][0]*tmp_translation[1])],
                                [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1] * tmp_translation[0] + tmp_t[1][1]*tmp_translation[1])],
                                [0,0,1]])

            #npc vehicle ranslation        
            for npc_list in self.all_object.npc_list:
                global_result=np.array([[npc_list.position.x],[npc_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :        
                    global_npc_info.append([npc_list.type,npc_list.position.x,npc_list.position.y,npc_list.velocity.x])
                    local_npc_info.append([npc_list.type,local_result[0][0],local_result[1][0],npc_list.velocity.x])

            #ped translation
            for ped_list in self.all_object.pedestrian_list:
                global_result=np.array([[ped_list.position.x],[ped_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_ped_info.append([ped_list.type,ped_list.position.x,ped_list.position.y,ped_list.velocity.x])
                    local_ped_info.append([ped_list.type,local_result[0][0],local_result[1][0],ped_list.velocity.x])

            #obs translation
            for obs_list in self.all_object.obstacle_list:
                global_result=np.array([[obs_list.position.x],[obs_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_obs_info.append([obs_list.type,obs_list.position.x,obs_list.position.y,obs_list.velocity.x])
                    local_obs_info.append([obs_list.type,local_result[0][0],local_result[1][0],obs_list.velocity.x])
                
        return global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info


    def calc_pure_pursuit(self,):
        self.lfd = self.lfd_gain * self.status_msg.velocity.x
        self.lfd = min(max(self.lfd, self.min_lfd), self.max_lfd)
        # rospy.loginfo(self.lfd)
        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]
        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                                    [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                                    [0, 0, 1]])
        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num,i in enumerate(self.path.poses) :
            path_point = i.pose.position

            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0]>0 :
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd :
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break

        theta = atan2(self.forward_point.y - vehicle_position.y, self.forward_point.x - vehicle_position.x) - self.vehicle_yaw
        steering = atan2(2.0 * self.vehicle_length * sin(theta), self.lfd)

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.38
        self.i_gain = 0.01
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel
        
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error


        return output

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))


            A = np.array(x_list)
            B = np.array(y_list)
            X = np.dot(np.linalg.inv(np.dot(A.T, A)), np.dot(A.T, B))
            r = sqrt(X[0]**2 + X[1]**2 - X[2])

            v_max = sqrt( r * 9.81 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan
    
class AdaptiveCruiseControl:
    def __init__(self, velocity_gain, distance_gain, time_gap, vehicle_length):
        self.npc_vehicle=[False,0]
        self.object=[False,0]
        self.Person=[False,0]
        self.velocity_gain = velocity_gain
        self.distance_gain = distance_gain
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length

        self.object_type = None
        self.object_distance = 0
        self.object_velocity = 0

    def check_object(self,ref_path, global_npc_info, local_npc_info, 
                                    global_ped_info, local_ped_info, 
                                    global_obs_info, local_obs_info):

        min_rel_distance=float('inf')
        if len(global_ped_info) > 0 :        
            for i in range(len(global_ped_info)):
                for path in ref_path.poses :      
                    if global_ped_info[i][0] == 0 : # type=0 [pedestrian]                    
                        dx = global_ped_info[i][1] - path.pose.position.x
                        dy = global_ped_info[i][2] - path.pose.position.y
                        dis = sqrt(dx**2 + dy**2)
                        if dis<2.5:                            
                            rel_distance = sqrt(pow(path.pose.position.x - global_ped_info[i][1], 2) + pow(path.pose.position.y - global_ped_info[i][2], 2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.Person=[True,i]

        if len(global_npc_info) > 0 :            
            for i in range(len(global_npc_info)):
                for path in ref_path.poses :      
                    if global_npc_info[i][0] == 1 : # type=1 [npc_vehicle] 
                        dx = global_npc_info[i][1] - path.pose.position.x
                        dy = global_npc_info[i][2] - path.pose.position.y
                        dis = sqrt(dx**2 + dy**2)
                        if dis<4.35:
                            rel_distance = dis
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.npc_vehicle=[True,i]
        
        if len(global_obs_info) > 0 :            
            for i in range(len(global_obs_info)):
                for path in ref_path.poses :      
                    if global_obs_info[i][0] == 2 : # type=2 [obstacle] 
                        dx = global_obs_info[i][1] - path.pose.position.x
                        dy = global_obs_info[i][2] - path.pose.position.y
                        dis = sqrt(dx**2 + dy**2)
                        if dis<4.35:
                            rel_distance = dis             
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                # self.object=[True,i] 

    def get_target_velocity(self, local_npc_info, local_ped_info, local_obs_info, ego_vel, target_vel): 
        out_vel =  target_vel
        default_space = 8
        time_gap = self.time_gap
        v_gain = self.velocity_gain
        x_errgain = self.distance_gain

        if self.npc_vehicle[0] and len(local_npc_info) != 0: #ACC ON_vehicle   
            print("ACC ON NPC_Vehicle")         
            front_vehicle = [local_npc_info[self.npc_vehicle[1]][1], local_npc_info[self.npc_vehicle[1]][2], local_npc_info[self.npc_vehicle[1]][3]]
            
            dis_safe = ego_vel * time_gap + default_space
            dis_rel = sqrt(pow(front_vehicle[0],2) + pow(front_vehicle[1],2))            
            vel_rel=((front_vehicle[2] / 3.6) - ego_vel)                        
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

            out_vel = ego_vel + acceleration      

        # if self.Person[0] and len(local_ped_info) != 0: #ACC ON_Pedestrian
        #     print("ACC ON Pedestrian")
        #     Pedestrian = [local_ped_info[self.Person[1]][1], local_ped_info[self.Person[1]][2], local_ped_info[self.Person[1]][3]]
            
        #     dis_safe = ego_vel* time_gap + default_space
        #     dis_rel = sqrt(pow(Pedestrian[0],2) + pow(Pedestrian[1],2))            
        #     vel_rel = (Pedestrian[2] - ego_vel)              
        #     acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

        #     out_vel = ego_vel + acceleration
   
        # if self.object[0] and len(local_obs_info) != 0: #ACC ON_obstacle     
        #     print("ACC ON Obstacle")                    
        #     Obstacle = [local_obs_info[self.object[1]][1], local_obs_info[self.object[1]][2], local_obs_info[self.object[1]][3]]
            
        #     dis_safe = ego_vel* time_gap + default_space
        #     dis_rel = sqrt(pow(Obstacle[0],2) + pow(Obstacle[1],2))            
        #     vel_rel = (Obstacle[2] - ego_vel)
        #     acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

        #     out_vel = ego_vel + acceleration           

        return out_vel * 3.6

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
