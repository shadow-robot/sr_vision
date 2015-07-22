#!/usr/bin/env python

import SimpleCV as sCV

from sr_object_segmentation import SrObjectSegmentation


class BlobsSegmentation(SrObjectSegmentation):
    """
    Segmentation based upon the search of blobs, with a SimpleCV algorithm
    """

    def __init__(self, image):
        """
        Initialize the blob segmentation object
        @param image - image to be segmented (numpy format)
        """
        SrObjectSegmentation.__init__(self)
        self.name = 'Finding blobs segmentation algorithm'
        self.points = self.segmentation(image)
        self.nb_segments = len(self.segmented_box)

    def segmentation(self, image):
        """
        Segmente the image into blobs
        @return - dictionnary of segments found with points coordinates
        """

        img = sCV.Image(image)
        blobs = img.findBlobs()

        if blobs:
            blobs.draw()
            blobs.show(width=2)
            img.addDrawingLayer(img.dl())
            img.show()

            # Return the whole of points from each blob as a dictionary
            blobs = blobs.sortArea()[::-1]
            dic = {}
            k = 0

            for blob in blobs:
                points = []
                xmin, ymin = blob.minX(), blob.minY()
                xmax, ymax = blob.maxX(), blob.maxY()
                for x in range(xmin, xmax):
                    for y in range(ymin, ymax):
                        if blob.contains((x, y)):
                            points.append((x, y))
                dic[k] = points
                k += 1

            # Sort by descending size of segments
            seg_by_length = sorted(dic.values(), key=len, reverse=True)

            for i in range(len(dic)):
                dic[i] = seg_by_length[i]

            return dic
        else:
            print 'no segment found'
