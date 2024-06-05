#!/usr/bin/env python

# firebase connect
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

# init api_key to connect with firebase 
import os
current_path = os.path.dirname(os.path.realpath(__file__))
cred = credentials.Certificate(f'{current_path}/firebase_key.json')
app = firebase_admin.initialize_app(cred)
db = firestore.client()


# create an Event for notifying main thread
import threading
callback_done = threading.Event()

# ROS
import rospy


class Realtime_listener:
    def __init__(self):
        rospy.init_node("realtime_listener", anonymous=True)
        
        # you can set a specific location like this...
        doc_ref = db.collection(u'Ego').document(u'Ego_status')
        # start realtime listener at doc_Ref
        doc_watch = doc_ref.on_snapshot(self.on_snapshot)
        

        self.change_type = False
        self.check_time = ""
        self.data = {}

        rate = rospy.Rate(2) # 2 times / 1 sec
        while not rospy.is_shutdown():
            if self.change_type == 'MODIFIED':
                print("-----------------------")
                print(f"modified at {self.check_time} !!")
                print(f"doc_ref data : {self.data}")
                print("-----------------------")

            elif self.change_type == "ADDED":
                print("-----------------------")
                print(f"added at {self.check_time} !!")
                print(f"doc_ref data : {self.data}")
                print("-----------------------")

            else:
                print('no changed...')


            self.change_type = False
            rate.sleep()


    # firebase reference (data type and fields)  >>  https://cloud.google.com/firestore/docs/reference/rpc/google.firestore.v1#documentchange
    # this function works only when there is a change at doc_Ref
    def on_snapshot(self, doc_snapshot, changes, read_time):
        
        # change type : 'ADDED', 'MODIFIED', 'REMOVED'
        for change in changes:
            self.change_type = change.type.name

        # you can check data here
        for doc in doc_snapshot:
            self.data =  doc.to_dict()

        # read time (based on the region where your DB is located)
        self.check_time = read_time

        callback_done.set()




if __name__ == "__main__":
    try:
        start = Realtime_listener()
    except rospy.ROSInterruptException:
        pass