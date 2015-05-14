#!/usr/bin/env python
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import SimpleCV as sCV


class SrObjectSegmentation:
    
    def __init__(self,image):  
        self.img=image
        self.points={}
        self.nb_segments=0
        

class BlobsSegmentation(SrObjectSegmentation):

    def segmentation(self):
        self.img = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
        img=sCV.Image(self.img)
        
        bin_img=img.binarize()
        inv_img=img.invert()
        blobs=inv_img.findBlobs() #better with binnarize or inverted image

        if blobs:
                blobs.draw()
                blobs.show(width=2)
                img.addDrawingLayer(inv_img.dl()) 
                img.show()
                
                #Return the whole of points from each blob as a dictionnary 
                blobs=blobs.sortArea()[::-1]
                dic={}
                id=0
                
                for blob in blobs:
                        points=[]
                        xmin,ymin=blob.minX(),blob.minY()
                        xmax,ymax=blob.maxX(),blob.maxY()
                        for x in range(xmin,xmax):
                                for y in range(ymin,ymax):
                                        if blob.contains((x,y)):
                                                points.append((x,y))
                        dic[id]=points
                        id+=1

                #Sort by descending size of segments
                seg_by_length=sorted(dic.values(),key=len,reverse=True)
                for i in range(len(dic)):
                        dic[i]=seg_by_length[i]

                self.points=dic
                self.nb_segments=len(dic)
        else:
                print 'no segment found'


class ColorSegmentation(SrObjectSegmentation):

    def segmentation(self,color):

        img=cv2.imread(self.img)

        # define the list of boundaries with this order: red,blue,yellow,gray
        h=0 #threshold
        boundaries = {
            'red':([17-h, 15-h, 100-h], [50+h, 56+h, 200+h]),
            'blue':([86-h, 31-h, 4-h], [220+h, 88+h, 50+h]),
            'yellow':([25-h, 146-h, 190-h], [62+h, 174+h, 250+h]),
            'gray':([103-h, 86-h, 65-h], [145+h, 133+h, 128+h])
        }

        pts=[]
        dic={}
        id=0

        (lower,upper)=boundaries[color]

        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(img, lower, upper)
        output = cv2.bitwise_and(img,img, mask = mask)

        # return a dictionnary with segment id and the coordinates of the points corresponding
        for y in range(len(mask)):
            x_seq=np.nonzero(mask[y])[0]
            while len(x_seq)!=0:
                for x in x_seq:
                    pts.append((x,y))
                break
            dic[id]=pts
            id+=1

        #Sort by descending size of segments
        seg_by_length=sorted(dic.values(),key=len,reverse=True)
        for i in range(len(dic)):
            dic[i]=seg_by_length[i][0]

        self.points=dic
        self.nb_segments=len(dic.values())


if __name__ == "__main__":

    ''' 
    try:
        SrObjectSegmentation('\DataSet\lvl1_1.png')
        rospy.init_node("sr_object_segmentation")
        rospy.spin()
    except rospy.ROSInterruptException:
        print 'Shutting down'
    cv2.destroyAllWindows()
    '''
