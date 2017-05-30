#!/usr/bin/env python
import rospy
import cv2
from pressure_pad.msg import pressure_read
from cv_bridge import CvBridge

image = cv2.imread("4.jpg")

rospy.init_node('custom_talker', anonymous=True)
pub = rospy.Publisher('/wallpusher/reading/left', pressure_read)
r = rospy.Rate(60) #10hz
msg = pressure_read()
print "HERE"
bridge = CvBridge()
msg.rivet_image = bridge.cv2_to_imgmsg(image, "bgr8")

while not rospy.is_shutdown():
    print "image sent"
    pub.publish(msg)
    r.sleep()

