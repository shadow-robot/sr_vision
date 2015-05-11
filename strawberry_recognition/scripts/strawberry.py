#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import SimpleCV as s_cv

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

    # define the boundaries for red
    h=15 #threshold
    red=[50,20,80]
    red_lower=np.array([x-h for x in red], dtype = "uint8")
    red_upper= np.array([x+h for x in red], dtype = "uint8")
   
    # find the color within the specified boundaries and apply the mask
    mask = cv2.inRange(cv_image, red_lower, red_upper)
    output = cv2.bitwise_and(cv_image, cv_image, mask = mask)


    '''
    ###### Contours finding ######
    
    #gray_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    gray_image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    gray_image=cv2.bilateralFilter(gray_image,2,100,100)
    edged=cv2.Canny(gray_image,30,200)
    # find contours in the edged image, keep only the largest
    # ones, and initialize our screen contour
    (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    #screenCnt = None
  
    strawberry=None
    for cnt in cnts:
      M=cv2.moments(cnt)
      cy = int(M['m10']/M['m00'])
      cx = int(M['m01']/M['m00'])
    
      if mask[cx,cy]==255:
        strawberry=cnt

    #print contours
    if strawberry is not None:
      cv2.drawContours(cv_image, strawberry, -1, (255, 255, 255), 1)
      print 'got it !'
    cv2.drawContours(cv_image, cnts, -1, (255, 255, 255), 1)
    '''


    # show the images
    cv2.imshow("images", np.hstack([cv_image, output]))
    cv2.waitKey(3)


    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,encoding="passthrough"))
      #self.image_pub.publish()
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
