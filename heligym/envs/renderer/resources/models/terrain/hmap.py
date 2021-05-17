import cv2
import numpy as np
import os

cwd = os.path.dirname(os.path.abspath(__file__))

img = cv2.imread(cwd + '/terrain_hmap.png',0)
rows,cols = img.shape


hmap = [rows, cols]

for i in range(rows):
    for j in range(cols):
        print(i, j)
        k = img[i,j] * 2685/255.0 # 2685 max height of world machine
        hmap.append(k)

np.save(cwd + '/hmap.npy', hmap)