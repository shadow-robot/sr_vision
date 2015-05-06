#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import SimpleCV as s_cv
from matplotlib.pyplot import *



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("red_objects",Image,queue_size=1)
    self.bridge=CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
    except CvBridgeError, e:
      print e

    ###### Color segmentation : red ######

    hsv=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
    hue,sat,val = cv2.split(hsv)
    lower = np.array([0,190,0],dtype=np.uint8)
    upper = np.array([5,255,255],dtype=np.uint8)
    mask = cv2.inRange(hsv,lower,upper)
    seg = cv2.bitwise_and(hsv, hsv, mask = mask)

    red_points=np.column_stack((np.nonzero(mask)[0],np.nonzero(mask)[1]))
   
    cv2.imshow('img',np.hstack([cv_image, seg]))   
    cv2.waitKey(3)



    try:
      self.image_pub.publish()
    except CvBridgeError, e:
      print e


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
