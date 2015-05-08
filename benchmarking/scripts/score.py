import cv2
import argparse
from matplotlib import pyplot as plt
import numpy as np
from SimpleCV import *
from read_seg import *
from math import sqrt



############################################################
#Score based upon the difference between numbers of segments
############################################################

def RightNumberSegments(algo_seg,ref_seg):
        print 'Found : ', len(algo_seg.keys())
        print 'Reference : ', len(ref_seg.keys())
        return abs(len(algo_seg)-len(ref_seg.keys()))



#################################################################################
####Score based upon the distance from the nearest point of the reference segment
#################################################################################

def Distance(algo_seg,ref_seg):
        min_seg=algo_seg
        max_seg=ref_seg
        if len(ref_seg.keys())<len(min_seg.keys()):
                min_seg=ref_seg
                max_seg=algo_seg

        #Correspondance between segments from ref and algo based upon number of pixels in common (seems cool, need more tests..)
        match={}
        corresp={}
        for i in range(len(min_seg.keys())):
                m={}
                maxi=[]
                for j in range(len(max_seg.keys())):
                        m[j]=len(set(min_seg[i]) & set(max_seg[j]))

                match[i]=m
                inv_m = dict(zip(m.values(), m.keys()))
                maxi=sorted(m.values())

                for k in range(1,len(m)):
                        if inv_m[maxi[-k]] not in corresp.values():
                                corresp[i]=inv_m[maxi[-k]]
                                break
                        else:
                                corresp[i]=inv_m[maxi[-(k+i)]]
                                break

        dist_seg=[]  
        for seg in range(1,len(min_seg.keys())): #not considering the background (bigger segment)
                seg1=min_seg[seg]
                seg2=max_seg[corresp[seg]]
                for point1 in seg1:
                        if point1 not in seg2:
                                dist=0
                                for point2 in seg2:
                                        d=sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
                                        if d>dist:
                                                dist=d
                                dist_seg.append(dist)
        
        return 0.02*sum(dist_seg)/len(dist_seg) #0.02 is arbitrary, need some adjustments..
                        
                                
                
#print RightNumberSegments(segments,ref_seg)
#print Distance(segments, ref_seg)




############################################################
#                     Total score
############################################################

def Score(segments,ref_seg):
        r=RightNumberSegments(segments,ref_seg)
        d=Distance(segments, ref_seg)
        return r+d

