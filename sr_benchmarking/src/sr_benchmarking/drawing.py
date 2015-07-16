#!/usr/bin/env python

import numpy as np
from PIL import Image
from PIL import ImageDraw


class ImagesTest(object):
    def __init__(self):
        """
        Initialize a test dataset images for benchmarking
        @attribute ref_seg - list of the dictionnaries corresponding to the
        segments (id as key and coordinates as
        values) for each image of the dataset
        @attribute nb_seg - total number of segments
        @attribute np_img - test images with a numpy format
        """

        self.name = None
        self.ref_seg = []
        self.pil_img = []
        self.np_img = []

    def get_pixels_coord(self):
        """
        Get the coordinates of the points in which segments
        @return - list of the dictionaries corresponding to the segments
        (id as key and coordinates as values) for
        each image of the dataset
        """

        ref_segments = []
        for img in self.pil_img:
            width = img.size[0]
            height = img.size[1]
            dic = {}
            blue_coord = []
            red_coord = []
            yellow_coord = []
            for y in range(height):
                for x in range(width):
                    if img.getpixel((x, y)) == (0, 0, 255):  # blue stuffs
                        blue_coord.append((y, x))
                    elif img.getpixel((x, y)) == (255, 0, 0):  # red stuffs
                        red_coord.append((y, x))
                    elif img.getpixel((x, y)) == (255, 255, 0):  # yellow
                        yellow_coord.append((y, x))

            if blue_coord:
                dic[0] = blue_coord
            if red_coord:
                dic[1] = red_coord
            if yellow_coord:
                if not red_coord:
                    dic[1] = yellow_coord
                else:
                    dic[2] = yellow_coord

            ref_segments.append(dic)

        return ref_segments

    def write_ref_file(self):
        """
        Write a segment reference text file for future usage.
        Coded as : "segment id"   "row id"   "min column"   "max column"
        """

        for k, img in enumerate(self.pil_img):
            width = img.size[0]
            height = img.size[1]
            path = 'DataSet/Reference_seg/' + self.name + str(k + 1) + '.seg'
            f = open(path, 'w')

            for row in range(height):
                column = 0
                while column < width:
                    for i, color in enumerate(
                            [(0, 0, 0), (0, 0, 255), (255, 0, 0),
                             (255, 255, 0), (0, 0, 0)]):
                        if img.getpixel((column, row)) == color:
                            column_min = column
                            column_tmp = column
                            while img.getpixel((column_tmp, row)) == color and\
                                            column_tmp < width - 1:
                                column_tmp += 1
                            if i == 4:
                                i = 0
                            f.write(str(i) + '\t' + str(row) + '\t' + str(
                                column_min) + '\t' + str(column_tmp) + '\n')
                            if column_tmp == width - 1:
                                column = 0
                                break
                            else:
                                column = column_tmp
                    break

            f.close()


class BasicTest(ImagesTest):
    """
    Basic dataset with simple images to segment
    """

    def __init__(self):
        """
        Initialize the Basic dataset object
        """
        ImagesTest.__init__(self)
        self.name = 'basic'
        self.pil_img = self.draw_basic_test()
        self.np_img = pil_to_array(self.pil_img)
        self.ref_seg = self.get_pixels_coord()

    def draw_basic_test(self):
        """
        Draw the images for the basic dataset
        @return - list of the images, with a PIL format
        """
        images = []

        # Draw a blue rectangle : test1_1
        images.append(Image.new("RGB", (640, 480), "black"))
        draw = ImageDraw.Draw(images[0])
        draw.rectangle([(500, 300), (200, 100)], fill=(0, 0, 255))

        # Draw red circles with a radius of 30px outer of the rectangle
        r = 30
        x, y = 100, 150
        images.append(Image.new("RGB", (640, 480), "black"))
        draw = ImageDraw.Draw(images[1])
        draw.rectangle([(500, 300), (200, 100)], fill=(0, 0, 255))
        draw.ellipse((x - r, y - r, x + r, y + r), fill=(255, 0, 0))

        # Draw red circles with a radius of 30px inner  of the rectangle
        r = 30
        x, y = 400, 200
        images.append(Image.new("RGB", (640, 480), "black"))
        draw = ImageDraw.Draw(images[2])
        draw.rectangle([(500, 300), (200, 100)], fill=(0, 0, 255))
        draw.ellipse((x - r, y - r, x + r, y + r), fill=(255, 0, 0))

        # Draw a yellow polygon outer of the rectangle : test 1_4
        images.append(Image.new("RGB", (640, 480), "black"))
        draw = ImageDraw.Draw(images[3])
        draw.rectangle([(500, 300), (200, 100)], fill=(0, 0, 255))
        draw.polygon([(50, 50), (100, 100), (200, 100), (20, 170)],
                     fill=(255, 255, 0))

        return images


class NoiseTest(ImagesTest):
    """
    Noise dataset with images to test the noise (gradually)
    """

    def __init__(self):
        ImagesTest.__init__(self)
        self.name = 'noise'
        self.pil_img = self.draw_noise_test()
        self.np_img = pil_to_array(self.pil_img)
        self.ref_seg = self.get_pixels_coord()

    def draw_noise_test(self):
        """
        Draw the images for the noise dataset
        @return - list of the images, with a PIL format
        """
        images = []

        # Draw a blue rectangle and 2 red circles with a radius smaller and
        # smaller (40,20,1 px)
        # Image 1
        images.append(Image.new("RGB", (640, 480), "black"))
        draw = ImageDraw.Draw(images[0])
        draw.rectangle([(500, 300), (200, 100)], fill=(0, 0, 255))
        r = 30
        for x, y in [(100, 150), (450, 400)]:
            draw.ellipse((x - r, y - r, x + r, y + r), fill=(255, 0, 0))
            draw.ellipse((x - r, y - r, x + r, y + r), fill=(255, 0, 0))

        # Image 2
        images.append(Image.new("RGB", (640, 480), "black"))
        draw = ImageDraw.Draw(images[1])
        draw.rectangle([(400, 250), (300, 150)], fill=(0, 0, 255))
        r = 10
        for x, y in [(100, 150), (450, 400)]:
            draw.ellipse((x - r, y - r, x + r, y + r), fill=(255, 0, 0))
            draw.ellipse((x - r, y - r, x + r, y + r), fill=(255, 0, 0))

        # Image 3
        images.append(Image.new("RGB", (640, 480), "black"))
        draw = ImageDraw.Draw(images[2])
        draw.rectangle([(320, 180), (350, 220)], fill=(0, 0, 255))
        r = 1
        for x, y in [(100, 150), (450, 400)]:
            draw.ellipse((x - r, y - r, x + r, y + r), fill=(255, 0, 0))
            draw.ellipse((x - r, y - r, x + r, y + r), fill=(255, 0, 0))

        return images


def pil_to_array(pil_images):
    """
    Convert PIL images to numpy format
    @return - list of numpy images
    """
    images = []
    for img in pil_images:
        images.append(np.array(img))

    return images
