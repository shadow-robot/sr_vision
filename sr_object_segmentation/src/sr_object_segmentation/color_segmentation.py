#!/usr/bin/env python

import Image

from sr_object_segmentation import *


class ColorSegmentation(SrObjectSegmentation):
    """
    Segmentation based upon a color segmentation
    """

    def __init__(self, image, color):
        """
        Initialize the color segmentation object with the color chosen to segmente as parameter
        @param image - image to be segmented (numpy format)
        @param color - name of the color (string) chosen to segmente the image
        """
        SrObjectSegmentation.__init__(self, image, {})
        self.color = color
        self.points = self.segmentation()
        self.nb_segments = len(self.points)

    def segmentation(self):
        """
        Segmente the image according to the color given as parameter
        @return - dictionnary of segments found with points coordinates
        """
        img = self.img

        '''
        # define the list of boundaries with this order: red,blue,yellow,gray
        boundaries = {
            'red': ([17, 15, 100], [50, 56, 200]),
            'blue': ([86, 31, 4], [220, 88, 50]),
            'yellow': ([25, 146, 190], [62, 174, 250]),
            'gray': ([103, 86, 65], [145, 133, 128]),
            'B': ([0, 0, 250], [0, 0, 255])
        }
        '''

        width = img.shape[0]
        height = img.shape[1]

        dic = {}
        dic_fin = {}

        main_colors = get_main_color(self.img, 4)
        for i, color in enumerate(main_colors):
            pts = []
            for x in range(width):
                for y in range(height):

                    if list(img[(x, y)]) == list(color):
                        pts.append((x, y))
            dic[i] = pts

        # Sort by descending size of segments
        seg_by_length = sorted(dic.values(), key=len, reverse=True)[1:]  # Remove the background (basic and noise test)
        for i in range(len(seg_by_length)):
            dic_fin[i] = seg_by_length[i]

        return dic_fin


def get_main_color(np_img, max_nb_col):
    pil_img = Image.fromarray(np_img)
    colors = pil_img.getcolors(256)
    sorted_colors = sorted(colors, key=lambda col: col[0:], reverse=True)
    if len(sorted_colors) < max_nb_col:
        nb_col = len(sorted_colors)
    else:
        nb_col = max_nb_col
    return [sorted_colors[:][i][1] for i in range(nb_col)]
