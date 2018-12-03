#!/usr/bin/env python
import os, sys
sys.path.insert(0, '../include/rrt-algorithms/examples')
import numpy as np
import pickle

import rospy

from std_msgs.msg import Bool
from rrt_ros_wrapper.msg import MatrixMap
from create_2d_data import Env2D
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D

import cv2

FILE_PATH = os.path.dirname(__file__)

path = "../include/rrt-algorithms/data"


class PubMap():
    #read data
    def __init__(self):
        print("start to load data...")
        self.bridge = CvBridge()

        self.data_path = os.path.join(FILE_PATH, path)
        self.env_list = os.listdir(self.data_path)
        self.map_pub = rospy.Publisher("map", MatrixMap, queue_size = 1)
        self.next_sub = rospy.Subscriber("map/next", Bool, self.pub_next_Cb, queue_size = 1)
        self.count = 0
   
    #load map data
    def load_data(self):
        map_msg = MatrixMap()
        if self.count < len(self.env_list):
            map2D = pickle.load(open(os.path.join(self.data_path, self.env_list[self.count],"env.pkl"),"rb"))
            pairs = pickle.load(open(os.path.join(self.data_path, self.env_list[self.count],"goals.pkl"), "rb"))

            map2D.map_matrix = map2D.map_matrix.astype(np.uint8)
            map_msg.map = self.bridge.cv2_to_imgmsg(map2D.map_matrix,"mono8")

            pose0 = Pose2D()
            pose1 = Pose2D()
            for pair in pairs:
                pose0.x = pair[0][0]
                pose0.y = pair[0][1]
                pose0.theta = 0
                map_msg.start.append(pose0)

                pose1.x = pair[1][0]
                pose1.y = pair[1][1]
                pose1.theta = 0
                map_msg.goal.append(pose1)
        else:
           map_msg.map = None
           map_msg.start = None
           map_msg.goal = None
        
        self.map_pub.publish(map_msg) 

    #publish
    def pub_next_Cb(self,msg):
        if msg.data == True:
            self.load_data()
            print ("true")


def main():
    pub_map = PubMap()
    rospy.init_node('map_publisher', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"


if __name__ == "__main__":
    main()
