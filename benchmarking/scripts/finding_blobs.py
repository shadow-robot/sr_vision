from SimpleCV import *


def FindingBlobs(img):
        bin_img=img.binarize()
        inv_img=img.invert()
        blobs=inv_img.findBlobs() #better with binnarize or inverted image
        #blobs=sorted(blobs,reverse=True)[:5]
        if blobs:
                blobs.draw()
                blobs.show(width=2)
                #print "Areas: ", blobs.area()
                #print "Angles: ", blobs.angle()
                #print "Centers: ", blobs.coordinates()
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
                        #print len(points),blob.area() #Not really working but no matter (blobs are just example)
                        dic[id]=points
                        id+=1

                #Sort by descending size of segments
                seg_by_length=sorted(dic.values(),key=len,reverse=True)
                for i in range(len(dic)):
                        dic[i]=seg_by_length[i]

                return dic
        else:
                print 'no segment found'
