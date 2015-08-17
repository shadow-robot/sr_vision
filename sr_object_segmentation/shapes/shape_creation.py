from optparse import OptionParser
import os

import numpy as np
import cv2

parser = OptionParser()
parser.add_option("-n", "--name", dest="name", help="Name of the new shape")
parser.add_option("-i", "--image", dest="img",
                  help="Path of the model shape image")
(options, args) = parser.parse_args()
name = options.name
path = options.img

model = cv2.imread(path, 0)

if model is not None:
    model = cv2.resize(model, (150, 150))
    model_thresh = cv2.adaptiveThreshold(model, 255,
                                         cv2.ADAPTIVE_THRESH_MEAN_C,
                                         cv2.THRESH_BINARY_INV, 11, 7)
    (model_cnt, _) = cv2.findContours(model_thresh.copy(),
                                      cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
    model_cnt = sorted(model_cnt, key=cv2.contourArea, reverse=True)[0]

    f = open(os.path.dirname(
        os.path.realpath(__file__)) + '/dataset/' + name + '.npy', 'w')
    np.save(f, model_cnt)
    f.close()

    outline = np.zeros(model.shape, dtype="uint8")
    cv2.drawContours(outline, [model_cnt], -1, 255, -1)
    cv2.imshow('shape model', outline)
    cv2.waitKey(0)

else:
    print 'Error : Unable to open file'
