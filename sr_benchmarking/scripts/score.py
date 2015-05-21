#!/usr/bin/env python

from math import sqrt

from sr_object_segmentation.sr_object_segmentation import SrObjectSegmentation
from sr_object_segmentation.blobs_segmentation import BlobsSegmentation
from sr_object_segmentation.color_segmentation import ColorSegmentation
from sr_benchmarking.drawing import BasicTest
from sr_benchmarking.drawing import NoiseTest


class TestObjectSegmentation(object):
    """
    Test class for object segmentation algorithms
    """

    def __init__(self, image, algo, ref_seg):
        """
        Initialize benchmarking comparison object
        @param image - image on which the benchmarking will be realized.
        @param algo - name of the class algorithm to be tested
        @param ref_seg - dictionnary containing the referencee segments as keys and coordinates of the points as values
        """
        print 'SEGMENTATION ...'
        self.algo = algo(image)

        # Get the theorical segmentation
        self.ref = SrObjectSegmentation(image, ref_seg)

    def test_number(self):
        """ 
        Verify that number of segments is correct
        @return - a score corresponding to the difference between number of segments found by the algorithm and the
        theorical, increase tenfold (arbitratry)
        """

        print 'NUMBER OF SEGMENTS TEST ...'
        res = abs(self.algo.nb_segments - self.ref.nb_segments)
        print 'Algorithm segments :', self.algo.nb_segments, 'Theorical segments :', self.ref.nb_segments, '\n'
        return 10 * res

    @property
    def test_distance(self):
        """ 
        If segments are not perfectly the same, this test measures the magnitude of the difference
        @return - a score corresponding to the minimal distance between a wrong point and the theorical,
        divided by 5 (arbitrary)
        """
        print 'DISTANCE TEST ...'

        min_seg = self.algo.points
        max_seg = self.ref.points
        if self.ref.nb_segments < self.algo.nb_segments:
            min_seg = self.ref.points
            max_seg = self.algo.points

        # Correspondance between segments from ref and algo based upon number of pixels in common
        match = {}
        corresp = {}

        for id_min_seg in range(len(min_seg)):
            m = {}
            for id_max_seg in range(len(max_seg)):
                m[id_max_seg] = len(set(min_seg[id_min_seg]) & set(max_seg[id_max_seg]))
            match[id_min_seg] = m
            inv_m = dict(zip(m.values(), m.keys()))
            maxi = sorted(m.values(), reverse=True)
            for k in range(len(m)):
                if inv_m[maxi[-k]] not in corresp.values():
                    corresp[id_min_seg] = inv_m[maxi[-k]]
                    break
                else:
                    corresp[id_min_seg] = inv_m[maxi[-(k + id_min_seg)]]
                    break
        dist_seg = []
        for seg in range(len(min_seg.values())):
            seg1 = min_seg[seg]
            seg2 = max_seg[corresp[seg]]
            print 'Number of wrong pixels : ', abs(len(seg1) - len(seg2))
            if seg1 == seg2 or seg1 == sorted(seg2) or seg2 == sorted(seg1):
                break
            else:
                for point1 in seg1:
                    if point1 not in seg2:
                        dist = 0
                        for point2 in seg2:
                            d = sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
                            if d > dist:
                                dist = d
                        dist_seg.append(dist)

        if len(dist_seg) == 0:
            return 0
        else:
            return 0.02 * sum(dist_seg) / len(dist_seg)  # 0.02 is arbitrary, need some adjustments..

    def score(self):
        """
        Calulate the final score
        @return - the sum of the two scores calculated by test_number and _test_distance methods
        """
        r = self.test_number()
        d = self.test_distance
        return r + d


def run_test(algo, dataset, writing=False):
    """
    Run the test for the algo given as parameter, on the dataset also given
    @param algo - Algorithm to be tested, from the "algos" list
    @param dataset - Dataset on which the algorithm will be tested, from the "tests" list
    @param writing - Boolean optional parameter, writing or not a resume text file of the scores
    """
    if writing:
        f = open(dataset.name + '.txt', 'w')

    for i, img in enumerate(dataset.np_img):
        s = dataset.name + str(i + 1)
        print '\n\n'
        print '##### Test ', s, '#####\n'
        if writing:
            f.write('\n\n')
            f.write('##### Test ' + s + '#####\n')
        test = TestObjectSegmentation(img, algo, dataset.ref_seg[i])
        score = test.score()
        print '\n'
        print 'TOTAL SCORE :', score
        if writing:
            f.write('\n')
            f.write('TOTAL SCORE :' + str(score))

    if writing:
        f.close()


if __name__ == '__main__':

    algos = [BlobsSegmentation, ColorSegmentation]
    tests = [BasicTest(), NoiseTest()]

    run_test(algos[0], tests[0])
