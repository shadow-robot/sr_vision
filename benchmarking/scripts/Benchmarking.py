import cv2
import argparse
from matplotlib import pyplot as plt
import numpy as np
from SimpleCV import *
from math import sqrt

from read_seg import *
from finding_blobs import *
from score import *

#Construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-r", "--reference", required = True,
	help = "Number of the reference image, from the dataset")
ap.add_argument("-s", "--segmented", required = True,
	help = "Path to the segmented image, with pixels colorized according to the average color of their attributed segment")
args = vars(ap.parse_args())



#Load the two segmentations
algo_seg=FindingBlobs(Image(args['segmented']))
ref_seg=read_seg_file(args['reference'])

#Calculate the score : smaller is better
print 'Benchmarking score : ',Score(algo_seg,ref_seg)


