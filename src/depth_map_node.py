#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from stereovision import Depthmap

class DepthMapNode:
    def __init__(self):
        rospy.init_node('depth_map_node', anonymous=True)
        
        self.left_sub = rospy.Subscriber('left_camera/image_raw', Image, self.left_callback)
        self.right_sub = rospy.Subscriber('right_camera/image_raw', Image, self.right_callback)
        
        self.depth_pub = rospy.Publisher('depth_map', Image, queue_size=10)
        
        self.bridge = CvBridge()
        self.depthmap = Depthmap()
        
        self.left_img = None
        self.right_img = None
    
    def left_callback(self, msg):
        self.left_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.compute_depth()
    
    def right_callback(self, msg):
        self.right_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.compute_depth()
    
    def compute_depth(self):
        if self.left_img is not None and self.right_img is not None:
            self.depthmap.left_img = self.left_img
            self.depthmap.right_img = self.right_img
            
            disparity = self.depthmap.compute_depth_map_sgbm()
            depth_msg = self.bridge.cv2_to_imgmsg(disparity, "32FC1")
            self.depth_pub.publish(depth_msg)

if __name__ == '__main__':
    try:
        node = DepthMapNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass