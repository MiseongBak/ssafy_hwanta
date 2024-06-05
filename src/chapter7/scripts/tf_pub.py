#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy, tf

from nav_msgs.msg import Odometry



class TF_publisher:
    def __init__(self):
        rospy.init_node("tf_publisher", anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.is_odom = False
        br = tf.TransformBroadcaster()
        self.br_param_translation = None
        self.br_param_rotation = None
        self.br_param_time = None
        self.br_param_child = "Ego"
        self.br_param_parent = "map"

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_odom == True:
                print("TF is currently being broadcast !")
                br.sendTransform(self.br_param_translation, 
                                 self.br_param_rotation,
                                 self.br_param_time, 
                                 self.br_param_child, 
                                 self.br_param_parent)
            else:
                print("waiting for data...")    
            rate.sleep()


    def odom_callback(self, odom_msg):
        self.is_odom = True
        pos_x = odom_msg.pose.pose.position.x
        pos_y = odom_msg.pose.pose.position.y
        ori_x = odom_msg.pose.pose.orientation.x
        ori_y = odom_msg.pose.pose.orientation.y
        ori_z = odom_msg.pose.pose.orientation.z
        ori_w = odom_msg.pose.pose.orientation.w

        self.br_param_translation = (pos_x, pos_y, 1)
        self.br_param_rotation = (ori_x, ori_y, ori_z, ori_w)
        self.br_param_time = rospy.Time.now()



if __name__ == '__main__':
    try:
        TF_publisher()
    except rospy.ROSInterruptException:
        pass