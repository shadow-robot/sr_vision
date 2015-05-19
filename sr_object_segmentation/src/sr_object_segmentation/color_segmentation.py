#!/usr/bin/env python
import cv2
import numpy as np
#import SimpleCV as sCV

from sr_object_segmentation import *

class ColorSegmentation(SrObjectSegmentation):
    """
    Segmentation based upon a color segmentation
    """

    def __init__(self,image,color):
        """
        Initialize the color segmentation object with the color chosen to segmente as parameter
        @param image - image to be segmented (numpy format)
        @param points - dictionnary with segments as keys and coordinates of the corresponding points as values (optionnal)
        """
        SrObjectSegmentation.__init__(self,image,{})
        self.color=color
        self.points=self.segmentation(color)
        self.nb_segments=len(self.points)
       


    def segmentation(self):
        """
        Segmente the image according to the color given as parameter
        @param color - name of the color (string) chosen to segmente the image
        @return - dictionnary of segments found with points coordinates
        """
        self.color=color
        img=cv2.imread(self.img)

        # define the list of boundaries with this order: red,blue,yellow,gray
        boundaries = {
            'red':([17, 15, 100], [50, 56, 200]),
            'blue':([86, 31, 4], [220, 88, 50]),
            'yellow':([25, 146, 190], [62, 174, 250]),
            'gray':([103, 86, 65], [145, 133, 128])
        }

        pts=[]
        dic={}
        k=0

        # create NumPy arrays from the boundaries
        lower = np.array(boundaries[color][0], dtype = "uint8")
        upper = np.array(boundaries[color][1], dtype = "uint8")

        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(img, lower, upper)
        #output = cv2.bitwise_and(img,img, mask = mask)

        # return a dictionnary with segment id and the coordinates of the points corresponding
        for y in range(len(mask)):
            x_seq=np.nonzero(mask[y])[0]
            while len(x_seq)!=0:
                for x in x_seq:
                    pts.append((x,y))
                break
            dic[k]=pts
            k+=1

        #Sort by descending size of segments
        seg_by_length=sorted(dic.values(),key=len,reverse=True)
        for i in range(len(dic)):
            dic[i]=seg_by_length[i][0]

        return dic


