import cv2
import numpy as np
import os

cwd = os.path.dirname(os.path.abspath(__file__))

img = cv2.imread(cwd + '/terrain_hmap.png',0)/255
# smooth height map since is might be too sharp.
kernel = np.ones((5,5),np.float)/25
img_smooth = cv2.filter2D(img,-1,kernel)

rows,cols = img_smooth.shape

hmap = [rows, cols]

for i in range(rows):
    for j in range(cols):
        #print(i, j)
        k = img_smooth[i,j] * 2685.0 # 2685 max height of world machine
        hmap.append(k)

np.save(cwd + '/hmap.npy', hmap)