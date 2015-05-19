#!/usr/bin/env python

from math import sqrt

from sr_object_segmentation.sr_object_segmentation import SrObjectSegmentation
from sr_object_segmentation.blobs_segmentation import BlobsSegmentation
from sr_object_segmentation.color_segmentation import ColorSegmentation
from sr_benchmarking.drawing import BasicTest 
from sr_benchmarking.drawing import NoiseTest 


class TestObjectSegmentation():
    """
    Test class for object segmentation algorithms
    """

    def __init__(self,img,algo,ref_seg,color=None):
        """
        Initialize benchmarking comparison object
        @param img - image on which the benchmarking will be realized. Can come from the dataset (Berkeley's) or drawn by the drawing script
        @param algo - name of the class algorithm to be tested
        @param color - color to be found by the Color segmentation (optional)     
        @param ref_seg - dictionnary containing the referencee segments as keys and coordinates of the points as values
        """
        print 'SEGMENTATION ...'
        if color:
            self.algo=algo(img,color)
        else:
            self.algo=algo(img)

        #Get the theorical segmentation
        self.ref=SrObjectSegmentation(img,ref_seg)

    def test_number(self):
        """ 
        Verify that number of segments is correct
        @return - a score corresponding to the difference between numer of segments found by the algorithm and the theorical, increase tenfold (arbitratry)
        """

        print 'NUMBER OF SEGMENTS TEST ...'
        res=abs(self.algo.nb_segments-self.ref.nb_segments)
        print 'Segments found by the algorithm :',self.algo.nb_segments,'Theorical number of segments :',self.ref.nb_segments,'\n'
        return 10*res


    def test_distance(self):
        """ 
        If segments are not perfectly the same, this test measures the magnitude of the difference
        @return - a score corresponding to the minimal distance between a wrong point and the theorical, divided by 5 (arbitrary)
        """
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
            m={}
            maxi=[]
            for j in range(len(max_seg)):
                m[j]=len(set(min_seg[i]) & set(max_seg[j]))
            match[i]=m
            inv_m = dict(zip(m.values(), m.keys()))
            maxi=sorted(m.values())
            for k in range(len(m)):
                if inv_m[maxi[-k]] not in corresp.values():
                    corresp[i]=inv_m[maxi[-k]]
                    break
                else:
                    corresp[i]=inv_m[maxi[-(k+i)]]
                    break
        dist_seg=[]  
        for seg in range(len(min_seg.values())): 
            seg1=min_seg[seg]
            seg2=max_seg[corresp[seg]]
            if seg1==seg2 or seg1==sorted(seg2) or seg2==sorted(seg1):
                break
            else:
                for point1 in seg1:
                    if point1 not in seg2:
                        dist=0
                        for point2 in seg2:
                            d=sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
                            if d>dist:
                                dist=d
                        dist_seg.append(dist)

        if len(dist_seg)==0:
            return 0 
        else:
            return 0.02*sum(dist_seg)/len(dist_seg) #0.02 is arbitrary, need some adjustments..
        


    def score(self):
        """
        Calulate the final score
        @return - the sum of the two scores calculated by test_number and _test_distance methods
        """
        r=self.test_number()
        d=self.test_distance()
        return r+d




if __name__ == '__main__':

    algos=[BlobsSegmentation,ColorSegmentation]
    color=['red','blue','yellow','gray']

    dataset=[BasicTest(),NoiseTest()]
    data=dataset[0]

    for i,img in enumerate(data.np_img):
        s=data.name+str(i+1)
        print '\n\n'
        print '##### Test ',s,'#####\n'
        test=TestObjectSegmentation(img,algos[0],data.ref_seg[i])
        score=test.score()
        print '\n'
        print 'TOTAL SCORE :',score
    

