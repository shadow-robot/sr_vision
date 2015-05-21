#!/usr/bin/env python

from PIL import Image

from sr_object_segmentation import SrObjectSegmentation


class ColorSegmentation(SrObjectSegmentation):
    """
    Segmentation based upon a color segmentation
    """

    def __init__(self, image):
        """
        Initialize the color segmentation object with the color chosen to segmente as parameter
        @param image - image to be segmented (numpy format)
        """
        SrObjectSegmentation.__init__(self, image, {})
        self.points = self.segmentation()
        self.nb_segments = len(self.points)

    def segmentation(self):
        """
        Segmente the image according to the color given as parameter
        @return - dictionnary of segments found with points coordinates
        """
        img = self.img

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
    """
    Get the main colors present in an image
    @param np_img: numpy image
    @param max_nb_col: number of maximum colors to be returned
    @return: a list of the main colors present in the image (RGB format)
    """

    pil_img = Image.fromarray(np_img)
    colors = pil_img.getcolors(256)
    sorted_colors = sorted(colors, key=lambda col: col[0:], reverse=True)
    if len(sorted_colors) < max_nb_col:
        nb_col = len(sorted_colors)
    else:
        nb_col = max_nb_col
    return [sorted_colors[:][i][1] for i in range(nb_col)]
