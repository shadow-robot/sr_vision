#!/usr/bin/env python
import cv2
import numpy as np
import SimpleCV as sCV


class SrObjectSegmentation:
    
    def __init__(self,image,points,nb_segments):  
        self.img=image
        self.points=points
        self.nb_segments=nb_segments
        

class BlobsSegmentation(SrObjectSegmentation):

    def segmentation(self):
      
        img=sCV.Image(self.img)

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

        self.points=dic
        self.nb_segments=len(dic.values())


