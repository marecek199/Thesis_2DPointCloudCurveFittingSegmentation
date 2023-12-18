
import numpy as np 
import matplotlib.pyplot as plt
import pickle 
from PIL import Image


pole_bodov = pickle.load(open('data_python/data_namerane_exp1.p','rb'))

maxX,maxY = 0,0
for bods in pole_bodov:
    x,y = bods[0],bods[1]
    maxX = x if x > maxX else maxX
    maxY = y if y > maxY else maxY

w = h = 50*(maxX//50 + 2) if maxX > maxY else 50*(maxY//50 + 2)


data_farba = np.zeros((w,h,3), dtype=np.uint8)
imagee = np.zeros((w,h), dtype=np.uint8)

print('Pocet bodov',str(len(pole_bodov)))

for bods in pole_bodov:
    x,y = bods[0],bods[1]
    data_farba[x][y][2] = 255
    imagee[w-y][x] = 255

Image.fromarray(imagee,'L').show()
