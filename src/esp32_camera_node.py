#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from esp32Driver import ESP32CameraManager

class ESP32CameraNode:
    def __init__(self):
        rospy.init_node('esp32_camera_node', anonymous=True)
        
        self.left_pub = rospy.Publisher('left_camera/image_raw', Image, queue_size=10)
        self.right_pub = rospy.Publisher('right_camera/image_raw', Image, queue_size=10)
        
        left_url = rospy.get_param('~left_camera_url', 'http://192.168.1.121')
        right_url = rospy.get_param('~right_camera_url', 'http://192.168.1.129')
        
        self.camera_manager = ESP32CameraManager(left_url, right_url)
        self.camera_manager.configure_cameras()
        
        self.bridge = CvBridge()
        
        self.rate = rospy.Rate(10)  # 10 Hz
    
    def run(self):
        while not rospy.is_shutdown():
            left_img, right_img = self.camera_manager.get_stereo_images()
            
            if left_img is not None and right_img is not None:
                left_msg = self.bridge.cv2_to_imgmsg(left_img, "bgr8")
                right_msg = self.bridge.cv2_to_imgmsg(right_img, "bgr8")
                
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ESP32CameraNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
