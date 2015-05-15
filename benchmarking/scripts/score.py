#!/usr/bin/env python

import random
import unittest 
import cv2
import numpy as np

import segmentation
import read_file
from math import sqrt
import drawing



class TestObjectSegmentationLvl1(unittest.TestCase):
    
    ''' Test class for object segmentation algorithms - Level 1'''

    def __init__(self,algo,img,type_test):
        ''' Initialization '''

        print 'SEGMENTATION ...'
        if 'blob' in algo:
            algo=segmentation.BlobsSegmentation(img)
            algo.segmentation()
        elif 'color' in algo:
            algo=segmentation.ColorSegmentation(img)
            algo.segmentation('blue')

        self.algo=algo
        
        #Get the theorical segmentation
        print 'Reading segmentation file'
        if type_test != '3096': #Berkeley dataset
            self.ref=read_file.read_seg_file(type_test)
        else:
            self.ref=read_file.read_seg_file(type_test,True)
        


    def test_number(self):
        print 'NUMBER OF SEGMENTS TEST ...'
        ''' Verify that number of segments is correct '''
        res=abs(self.algo.nb_segments-self.ref.nb_segments)
        print 'Segments found by the algorithm :',self.algo.nb_segments,'Theorical number of segments :',self.ref.nb_segments,'\n'
        return 10*res


    def test_distance(self):

        print 'DISTANCE TEST ...'
        min_seg=self.algo.points
        max_seg=self.ref.points
        if self.ref.nb_segments<self.algo.nb_segments:
            min_seg=self.ref.points
            max_seg=self.algo.points
        
        #Correspondance between segments from ref and algo based upon number of pixels in common (seems cool, need more tests..)
        match={}
        corresp={}
        
        for i in range(len(min_seg)):
            print 'len min seg',len(min_seg[i])
            m={}
            maxi=[]
            for j in range(len(max_seg)):
                print 'len max seg j',len(max_seg[j])
                m[j]=len(set(min_seg[i]) & set(max_seg[j]))
            print 'm',m
            match[i]=m
            inv_m = dict(zip(m.values(), m.keys()))
            maxi=sorted(m.values())
            print 'match[i]',match[i]
            print 'inv_m',inv_m
            print 'maxi',maxi
            for k in range(len(m)):
                if inv_m[maxi[-k]] not in corresp.values():
                    corresp[i]=inv_m[maxi[-k]]
                    break
                else:
                    corresp[i]=inv_m[maxi[-(k+i)]]
                    break
        print corresp
        dist_seg=[]  
        for seg in range(len(min_seg.values())): 
            seg1=min_seg[seg]
            seg2=max_seg[corresp[seg]]
            print len(seg1),len(seg2)
            if seg1==seg2 or seg1==sorted(seg2) or seg2==sorted(seg1):
                break
            else:
                for point1 in seg1:
                    if point1 not in seg2:
                        dist=0
                        for point2 in seg2:
                            d=sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
                            #print d
                            if d>dist:
                                dist=d
                        dist_seg.append(dist)

        if len(dist_seg)==0:
            return 0 
        else:
            return 0.02*sum(dist_seg)/len(dist_seg) #0.02 is arbitrary, need some adjustments..
        


    def score(self):
        r=self.test_number()
        d=self.test_distance()
        return r+d




if __name__ == '__main__':

    algo=['blob','color'] #ColorSegmentation algorithm needs some corrections..
    
    type_test=['basic','noise','3096']
    images=drawing.ImagesTest()

    
    for i,img in enumerate(images.basic_test):
        s=str(type_test[0])+str(i+1)
        print '\n\n'
        print '##### Test ',s,'#####\n'
        test=TestObjectSegmentationLvl1(algo[0],img,s)
        score=test.score()
        print '\n'
        print 'TOTAL SCORE :',score
    
    '''
    test=TestObjectSegmentationLvl1(algo[0],images.basic_test[1],'basic2')
    score=test.score()
    print 'Score',score
    '''
