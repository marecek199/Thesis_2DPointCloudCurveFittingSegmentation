



import numpy as np 
import matplotlib.pyplot as plt
import pickle 
import math

file = open('data_python/data_namerane_polarne_vsetky.txt','r')

new_data = []
pomocne_data = []

for _ in range(107):
    file.readline(1)
    data = file.readline().split(',')

    for i in range(0,len(data)-2,2):
        pomocne_data.append([float(data[i][1:]), float(data[i+1][:-1])])

    pomocne_data.append([float(data[-2][1:]), float(data[-1][0:-3])])    
    
    # print('Celkovo',str(len(pomocne_data))+' bodov')
    new_data.append(pomocne_data)
    pomocne_data=[]


pickle.dump(new_data,open('data_python/data_namerane_polarne_vsetky.p','wb'))

# plt.ion()

# fi=0
# plt.axis([-1,3,-1.5,1.8])
# plt.plot(0,0,'om')
# for bod in new_data[0]:
#     if bod[1]==0 :
#         fi -= bod[0]
#     else:
#         fi -= bod[0]
#         r = bod[1]        
#         x_cartes = -r*math.cos(fi)
#         y_cartes = r*math.sin(fi)
#         plt.plot(x_cartes,y_cartes,'.b')
#         plt.pause(0.01)
# plt.ioff()

# # plt.show()

