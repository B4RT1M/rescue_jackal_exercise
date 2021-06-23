# -*- coding: utf-8 -*-
"""
@author: Lars Schilling

"""
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt

#add imports for the pathfinding package

def callback(msg):
        print('Recieved map data')
        bridge = CvBridge()
        data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height_map=np.array(data, dtype=np.uint)

        #add a nonlinearity to the height map to create a height cost map (low points very low cost, high points very high cost)

        #create a distance cost map with the shape of height_costmap
        #pixels around the center should have low cost and get higher at the edges

        #define a combined cost map based height and distance cost map
        #this could be a sum or multiplication

        #implement the AStarFinder from the pathfinding package to the combined cost map
        #to find a path from the center of the image (30,30) to the upper edge of the image (30,0)


        #plot your height, distance and combined cost map, as well as the astar path
        plt.imshow(height_map)
        plt.show()
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:

        rospy.init_node('elevation_path')
        sub=rospy.Subscriber("/grid_map_image", Image, callback, queue_size=1)
        rospy.spin()
    except KeyboardInterrupt or rospy.ROSInterruptException: pass
