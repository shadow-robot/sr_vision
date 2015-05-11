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
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1)
    self.bridge=CvBridge()

    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
    except CvBridgeError, e:
      print e

    ###### Color segmentation ######

    # define the list of boundaries with this order: red,blue,yellow,gray
    h=10 #threshold
    boundaries = [
	([17-h, 15-h, 100-h], [50+h, 56+h, 200+h]),
	([86-h, 31-h, 4-h], [220+h, 88+h, 50+h]),
	([25-h, 146-h, 190-h], [62+h, 174+h, 250+h]),
	([103-h, 86-h, 65-h], [145+h, 133+h, 128+h])
    ]

    cv_image=cv2.resize(cv_image,(cv_image.shape[1]/2,cv_image.shape[0]/2))
    compare=[]  
    
    # loop over the boundaries
    for (lower, upper) in boundaries:
      # create NumPy arrays from the boundaries
      lower = np.array(lower, dtype = "uint8")
      upper = np.array(upper, dtype = "uint8")
      
      # find the colors within the specified boundaries and apply
      # the mask
      mask = cv2.inRange(cv_image, lower, upper)
      output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
      
      # show the images
      compare.append(np.hstack([cv_image, output]))

    cv2.imshow("images", np.vstack(compare))
    cv2.waitKey(3)



    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,encoding="passthrough"))
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
