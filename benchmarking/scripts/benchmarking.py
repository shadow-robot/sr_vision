import cv2
from SimpleCV import *
import numpy as np
from matplotlib import pyplot as plt
from collections import Counter

#load the image
img = cv2.imread('boxes.png') 

# define the list of boundaries with this order: red,blue,green,gray
h=0
boundaries = [
    ([255, 0, 0], [255, 0, 0]),
    ([0, 255, 0], [0, 255, 0]),
    ([0, 0, 255], [0, 0, 255]),
    ([102, 102, 102], [102, 102, 102])
]

img=cv2.resize(img,(img.shape[1]/3,img.shape[0]/3))
compare=[]  

# loop over the boundaries
for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")
    
    # find the colors within the specified boundaries and apply the mask
    mask = cv2.inRange(img,lower,upper)
    
    output = cv2.bitwise_and(img, img, mask=mask)
   
    #score : how many pixels found
    pixels=[]
    pixels.append(np.count_nonzero(mask))

    # show the images
    print pixels
    compare.append(np.hstack([img, output]))

while True:
    cv2.imshow("images", np.vstack(compare))
    cv2.waitKey(3)


'''

plt.subplot(211),plt.imshow(img)
plt.title('Input Image'), plt.xticks([]), plt.yticks([])
plt.subplot(212),plt.imshow(img_cut)
plt.title('Grab cut'), plt.xticks([]), plt.yticks([])
plt.show()

'''



