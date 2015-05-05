#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import SimpleCV as s_cv
#from matplotlib import pyplot as plt

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1)
    self.bridge=CvBridge()
    #cv2.namedWindow("Image window", 1)

    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='rgb8')
    except CvBridgeError, e:
      print e

    ###### Color segmentation ######

    boundaries = [
      ([17, 15, 100], [50, 56, 200]),
      ([86, 31, 4], [220, 88, 50]),
      ([25, 146, 190], [62, 174, 250]),
      ([103, 86, 65], [145, 133, 128])
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

    

    #Draw circle
    '''
    (rows,cols) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (int(rows/2),int(cols/2)), 100, 255,thickness=10)
    '''

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)


    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,encoding="passthrough"))
    except CvBridgeError, e:
      print e

'''
class segmentation:

  def __init__(self):
    k=s_cv.Kinect()
    self.image=k.getImage()

  def callback(self):
    ### Object segmentation ###
    features=self.image.findHaarFeatures('face.xml')
    if features:
      #Get the largest one
      features=features.sortArea()
      bigFeatures=features[-1]
      #Draw a green box around 
      bigFeatures.draw()
    print type(self.image)
    #cv2.imshow("Image window", self.image)
    #cv2.waitKey(3)
    self.image.show()

'''


def main(args):
  ic = image_converter()
  #seg=segmentation()
  #seg.callback()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
